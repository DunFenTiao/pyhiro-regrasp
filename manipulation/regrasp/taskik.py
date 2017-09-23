#!/usr/bin/python

import os
import itertools

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from panda3d.bullet import BulletWorld
from panda3d.core import *
from shapely.geometry import LinearRing
from shapely.geometry import Point
from shapely.geometry import Polygon

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
import trimesh
from pandaplotutils import pandageom as pg
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from mpl_toolkits.mplot3d import art3d as mc3d
from operator import add
from robotsim.nextage import nxt
from robotsim.hrp5 import hrp5
from robotsim.nextage import nxt
from database import dbaccess as db

import networkx as nx
import math
import random

# regriptpp means regrip using tabletop placements
class Taskik():

    def __init__(self, objpath, robot, handpkg, gdb, offset = -55):
        self.objtrimesh=trimesh.load_mesh(objpath)
        self.handpkg = handpkg
        self.handname = handpkg.getHandName()

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # regg = regrip graph
        self.regg = nx.Graph()

        self.ndiscreterot = 0
        self.nplacements = 0
        self.globalgripids = []

        # for removing the grasps at start and goal
        self.robothand = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])

        # plane to remove hand
        self.bulletworld = BulletWorld()
        self.planebullnode = cd.genCollisionPlane(offset = offset)
        self.bulletworld.attachRigidBody(self.planebullnode)

        # add tabletop plane model to bulletworld
        # dont forget offset
        # this_dir, this_filename = os.path.split(__file__)
        # ttpath = Filename.fromOsSpecific(os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "supports", "tabletop.egg"))
        # self.ttnodepath = NodePath("tabletop")
        # ttl = loader.loadModel(ttpath)
        # ttl.instanceTo(self.ttnodepath)

        self.startnodeids = None
        self.goalnodeids = None
        self.shortestpaths = None

        self.gdb = gdb
        self.robot = robot

        # load retraction distances
        self.rethandx, self.retworldz, self.retworlda, self.worldz = self.gdb.loadIKRet()
        # worlda is default for the general grasps on table top
        # for assembly at start and goal, worlda is computed by assembly planner
        self.worlda = Vec3(0,0,1)

        # loadfreeairgrip
        self.__loadFreeAirGrip()


    def __loadFreeAirGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: weiwei
        date: 20170110
        """

        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname, self.handname)
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripid = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]


    def savestartik(self,startrotmat4,startid,base):
        nodeidofglobalidinstart = {}
        # the startnodeids is also for quick access
        self.startnodeids = []
        for j, rotmat in enumerate(self.freegriprotmats):
            # print j, len(self.freegriprotmats)
            # # print rotmat
            ttgsrotmat = rotmat * startrotmat4
            # for collision detection, we move the object back to x=0,y=0
            ttgsrotmatx0y0 = Mat4(startrotmat4)
            ttgsrotmatx0y0.setCell(3, 0, 0)
            ttgsrotmatx0y0.setCell(3, 1, 0)
            ttgsrotmatx0y0 = rotmat * ttgsrotmatx0y0
            # check if the hand collide with tabletop
            # tmphnd = self.robothand
            tmphnd = self.handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
            initmat = tmphnd.getMat()
            initjawwidth = tmphnd.jawwidth
            # set jawwidth to 80 to avoid collision with surrounding obstacles
            # set to gripping with is unnecessary
            tmphnd.setJawwidth(80)
            tmphnd.setMat(ttgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)

            if not result.getNumContacts():
                ttgscct0 = startrotmat4.xformPoint(self.freegripcontacts[j][0])
                ttgscct1 = startrotmat4.xformPoint(self.freegripcontacts[j][1])
                ttgsfgrcenter = (ttgscct0 + ttgscct1) / 2
                handx = ttgsrotmat.getRow3(0)
                ttgsfgrcenterhandx = ttgsfgrcenter + handx * self.rethandx
                # handxworldz is not necessary for start
                # ttgsfgrcenterhandxworldz = ttgsfgrcenterhandx + self.worldz*self.retworldz
                ttgsfgrcenterworlda = ttgsfgrcenter + self.worlda * self.retworlda
                ttgsfgrcenterworldaworldz = ttgsfgrcenterworlda + self.worldz * self.retworldz
                ttgsjawwidth = self.freegripjawwidth[j]
                ttgsidfreeair = self.freegripid[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                # handxworldz is not necessary for start
                # ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                # print "solving starting iks"
                ikc = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np)
                ikcx = self.robot.numikr(ttgsfgrcenternp_handx, ttgsrotmat3np)
                ikca = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np)
                ikcaz = self.robot.numikr(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np)

                feasibility = {}
                feasibility_handx = {}
                #feasibility_handxworldz = {}
                feasibility_worlda = {}
                feasibility_worldaworldz = {}

                if ikc is not None:
                    feasibility[j] = 'True'
                else:
                    feasibility[j] = 'False'
                if ikcx is not None:
                    feasibility_handx[j] = 'True'
                else:
                    feasibility_handx[j] = 'False'

                if ikca is not None:
                    feasibility_worlda[j] = 'True'
                else:
                    feasibility_worlda[j] = 'False'
                if ikcaz is not None:
                    feasibility_worldaworldz[j] = 'True'
                else:
                    feasibility_worldaworldz[j] = 'False'

                sql = "INSERT IGNORE INTO ik_start(idstart,idfreeairgrip, feasibility, feasibility_handx,  \
                                              feasibility_worlda, feasibility_worldaworldz) VALUES (%d,%d, '%s', '%s', '%s', '%s')" % \
                      (startid, j, feasibility[j], feasibility_handx[j],\
                       feasibility_worlda[j], feasibility_worldaworldz[j])
                self.gdb.execute(sql)
    def selectstartik(self,startrotmat4,startid,j,base):
        sql = "SELECT ik_start.idstart,ik_start.idfreeairgrip,ik_start.feasibility, ik_start.feasibility_handx,  \
                     ik_start.feasibility_worlda, ik_start.feasibility_worldaworldz FROM ik_start\
                        WHERE  ik_start.idstart=%d AND ik_start.idfreeairgrip=%d  \
                           AND ik_start.feasibility='True' AND ik_start.feasibility_handx='True'  \
                        AND ik_start.feasibility_worlda='True' AND ik_start.feasibility_worldaworldz='True' " % (startid,j)
        result=self.gdb.execute(sql)
        if len(result)!=0:

            return True
        else:
            #print "no ik this start this grip"
            return False

    def savegoalik(self, goalrotmat4,goalid, base):
        ### goal
        # the node id of a globalgripid in goalnode, for quick setting up edges
        nodeidofglobalidingoal = {}
        # the goalnodeids is also for quick access
        self.goalnodeids = []
        for j, rotmat in enumerate(self.freegriprotmats):
            # print j, len(self.freegriprotmats)
            ttgsrotmat = rotmat * goalrotmat4
            # for collision detection, we move the object back to x=0,y=0
            ttgsrotmatx0y0 = Mat4(goalrotmat4)
            ttgsrotmatx0y0.setCell(3, 0, 0)
            ttgsrotmatx0y0.setCell(3, 1, 0)
            ttgsrotmatx0y0 = rotmat * ttgsrotmatx0y0
            # check if the hand collide with tabletop
            tmphnd = self.robothand
            initmat = tmphnd.getMat()
            initjawwidth = tmphnd.jawwidth
            tmphnd.setJawwidth(self.freegripjawwidth[j])
            tmphnd.setMat(ttgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                ttgscct0 = goalrotmat4.xformPoint(self.freegripcontacts[j][0])
                ttgscct1 = goalrotmat4.xformPoint(self.freegripcontacts[j][1])
                ttgsfgrcenter = (ttgscct0 + ttgscct1) / 2
                handx = ttgsrotmat.getRow3(0)
                ttgsfgrcenterhandx = ttgsfgrcenter + handx * self.rethandx
                ttgsfgrcenterhandxworldz = ttgsfgrcenterhandx + self.worldz * self.retworldz
                ttgsfgrcenterworlda = ttgsfgrcenter + self.worlda * self.retworlda
                ttgsfgrcenterworldaworldz = ttgsfgrcenterworlda + self.worldz * self.retworldz
                ttgsjawwidth = self.freegripjawwidth[j]
                ttgsidfreeair = self.freegripid[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                # print "solving goal iks"
                ikc = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np)
                ikcx = self.robot.numikr(ttgsfgrcenternp_handx, ttgsrotmat3np)
                ikcxz = self.robot.numikr(ttgsfgrcenternp_handxworldz, ttgsrotmat3np)
                ikca = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np)
                ikcaz = self.robot.numikr(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np)


                feasibility = {}
                feasibility_handx = {}
                feasibility_handxworldz = {}
                feasibility_worlda = {}
                feasibility_worldaworldz = {}

                if ikc is not None:
                    feasibility[j] = 'True'
                else:
                    feasibility[j] = 'False'
                if ikcx is not None:
                    feasibility_handx[j] = 'True'
                else:
                    feasibility_handx[j] = 'False'
                if ikcxz is not None:
                    feasibility_handxworldz[j] = 'True'
                else:
                    feasibility_handxworldz[j] = 'False'
                if ikca is not None:
                    feasibility_worlda[j] = 'True'
                else:
                    feasibility_worlda[j] = 'False'
                if ikcaz is not None:
                    feasibility_worldaworldz[j] = 'True'
                else:
                    feasibility_worldaworldz[j] = 'False'

                sql = "INSERT IGNORE INTO ik_goal(idgoal,idfreeairgrip, feasibility, feasibility_handx, feasibility_handxworldz, \
                                                            feasibility_worlda, feasibility_worldaworldz) VALUES (%d,%d, '%s', '%s', '%s', '%s', '%s')" % \
                      (goalid, j, feasibility[j], feasibility_handx[j], \
                       feasibility_handxworldz[j], \
                       feasibility_worlda[j], feasibility_worldaworldz[j])
                self.gdb.execute(sql)
    def selectgoalik(self, goalrotmat4, goalid, j, base):

        sql = "SELECT ik_goal.idgoal,ik_goal.idfreeairgrip,ik_goal.feasibility, ik_goal.feasibility_handx,ik_goal.feasibility_handxworldz,  \
                             ik_goal.feasibility_worlda, ik_goal.feasibility_worldaworldz FROM ik_goal\
                            WHERE  ik_goal.idgoal=%d AND ik_goal.idfreeairgrip=%d AND\
                            ik_goal.feasibility='True' AND ik_goal.feasibility_handx='True' AND ik_goal.feasibility_handxworldz='True' \
                            AND ik_goal.feasibility_worlda='True' AND ik_goal.feasibility_worldaworldz='True'  \
                             " % (goalid, j)
        result = self.gdb.execute(sql)
        if len(result)!= 0:
            return True
        else:
            #print "no ik this start this grip"
            return False



if __name__=='__main__':
    gdb = db.GraspDB()
    nxtrobot = nxt.NxtRobot()
    handpkg = rtq85nm

    base = pandactrl.World(camp=[5000, -1000, 1000], lookatp=[0, 0, 0])

    # ttube.stl
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "ttube.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "tool.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planewheel.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planelowerbody.stl")
    # objpath = os.path.join(os.path.split(os.path.split(this_dir)[0])[0], "grip", "objects", "planefrontstay.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planerearstay.stl")
    this_dir = "E:/project/manipulation/regrasp_onworkcell/dropsimulation"
    #objpath = os.path.join(this_dir, "objects", "t2tube.stl")
    objpath = os.path.join(this_dir, "objects", "CameraFrontCase.stl")
    workcellpath = os.path.join(this_dir, "objects", "ipadbox.stl")
    dbobjname = dbobjname = os.path.splitext(os.path.basename(objpath))[0]

    taskik= Taskik(objpath, nxtrobot, handpkg,gdb)

    # start and goal
    # table top placements


    sql = "SELECT startgoal.idstartgoal,startgoal.rotmat,startgoal.idfreetabletopplacement FROM startgoal,freetabletopplacement,object WHERE \
                         startgoal.idfreetabletopplacement=freetabletopplacement.idfreetabletopplacement AND \
                          freetabletopplacement.idobject=object.idobject AND object.name LIKE '%s'" % dbobjname
    result = gdb.execute(sql)
    result = np.asarray(result)
    idsglist = [int(x) for x in result[:, 0]]
    sgrotmatlist = [dc.strToMat4(x) for x in result[:, 1]]
    idfreeplacementlist = [int(x) for x in result[:, 2]]

    for idstart, start in zip(idsglist, sgrotmatlist):
        print "idstart",idstart
        taskik.savestartik(start, idstart, base)
    for idgoal, goal in zip(idsglist, sgrotmatlist):
        print "idgoal",idgoal
        taskik.savegoalik(goal,idgoal,base)

    base.run()