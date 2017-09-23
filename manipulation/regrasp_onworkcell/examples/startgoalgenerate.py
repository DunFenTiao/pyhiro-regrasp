#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from panda3d.bullet import BulletWorld
from panda3d.core import *

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
from manipulation.regrasp_onworkcell.grip import freetabletopplacement as tp
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from robotsim.nextage import nxt
from robotsim.hrp5 import hrp5
from robotsim.hrp5n import hrp5n
from robotsim.hrp2k import hrp2k
from database import dbaccess as db
import trimesh
import time

class TablePlacements(object):
    """
    manipulation.freetabletopplacement doesn't take into account
    the position and orientation of the object
    it is "free" in position and rotation around z axis
    in contrast, each item in regrasp.tabletopplacements
    has different position and orientation
    it is at a specific pose in the workspace
    To clearly indicate the difference, "free" is attached
    to the front of "freetabletopplacement"
    "s" is attached to the end of "tabletopplacements"
    """

    def __init__(self, objpath, handpkg):
        """
        initialization

        :param objpath: path of the object

        author: weiwei
        date: 20161215, osaka
        """

        self.objtrimesh = trimesh.load_mesh(objpath)
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()

    def saveToDB(self, positionlist, gdb, discretesize=4):
        """

        :param positionlist: a list of positions to place the object one the table
        :param discretesize: the discretization of rotation angles around z axis
        :return:

        author: weiwei
        date: 20161215, osaka
        """

        # save discretiezed angle
        sql = "SELECT * FROM angle"
        result = gdb.execute(sql)
        if len(result) == 0:
            sql = "INSERT INTO angle(value) VALUES "
            for i in range(discretesize):
                sql += "("+str(360*i*1.0/discretesize)+"), "
            sql = sql[:-2]+";"
            gdb.execute(sql)
        else:
            print "Angles already set!"

        # save tabletopplacements
        sql = "SELECT idstartgoal FROM startgoal,freetabletopplacement,object WHERE \
                startgoal.idfreetabletopplacement=freetabletopplacement.idfreetabletopplacement AND \
                 freetabletopplacement.idobject=object.idobject AND object.name LIKE '%s'" % self.dbobjname
        result = gdb.execute(sql)
        if len(result) == 0:
            # 1) select the freetabletopplacement
            sql = "SELECT freetabletopplacement.idfreetabletopplacement, freetabletopplacement.rotmat \
                        FROM freetabletopplacement,object WHERE freetabletopplacement.idobject = object.idobject \
                        AND object.name LIKE '%s'" % self.dbobjname
            result = gdb.execute(sql)
            if len(result) == 0:
                raise ValueError("Plan the freetabletopplacement first!")
            result = np.asarray(result)
            idfreelist = [int(x) for x in result[:, 0]]
            rotmatfreelist = [dc.strToMat4(x) for x in result[:, 1]]
            # 2) select the angle
            sql = "SELECT angle.idangle,angle.value FROM angle"
            result = np.asarray(gdb.execute(sql))
            idanglelist = [int(x) for x in result[:, 0]]
            anglevaluelist = [float(x) for x in result[:, 1]]
            # 3) save to database
            sql = "INSERT INTO startgoal(rotmat, tabletopposition, idangle, idfreetabletopplacement) VALUES "
            for ttoppos in positionlist:
                ttoppos = Point3(ttoppos[0], ttoppos[1], ttoppos[2])
                for idfree, rotmatfree in zip(idfreelist, rotmatfreelist):
                    for idangle, anglevalue in zip(idanglelist, anglevaluelist):
                        rotangle = anglevalue
                        rotmat = rm.rodrigues([0, 0, 1], rotangle)
                        rotmat4 = pg.cvtMat4(rotmat, ttoppos)
                        varrotmat = rotmatfree * rotmat4
                        sql += "('%s', '%s', %d, %d), " % \
                              (dc.mat4ToStr(varrotmat), dc.v3ToStr(ttoppos), idangle, idfree)
            sql = sql[:-2]+";"
            gdb.execute(sql)
        else:
            print "startgoal already exist!"

        print "Save to DB done!"






if __name__ == '__main__':
    nxtrobot = nxt.NxtRobot()
    hrp5robot = hrp5.Hrp5Robot()
    hrp5n = hrp5n.Hrp5NRobot()
    hrp2k = hrp2k.Hrp2KRobot()

    base = pandactrl.World(camp=[1000,400,1000], lookatp=[400,0,0])
    #this_dir, this_filename = os.path.split(__file__)
    #objpath = os.path.join(os.path.split(this_dir)[0] + os.sep, "grip", "objects", "t2tube.stl")
    this_dir = "E:/project/manipulation/regrasp_onworkcell/dropsimulation"
    #objpath = os.path.join(this_dir, "objects", "t2tube.stl")
    objpath = os.path.join(this_dir, "objects", "CameraFrontCase.stl")
    workcellpath = os.path.join(this_dir, "objects", "ipadbox.stl")

    #from manipulation.grip.hrp5three import hrp5threenm
    #handpkg = hrp5threenm
    handpkg = rtq85nm
    print objpath
    tps = TablePlacements(objpath, handpkg)

    #plot obj and its convexhull
    geom = pg.packpandageom(tps.objtrimesh.vertices,
                                   tps.objtrimesh.face_normals,
                                   tps.objtrimesh.faces)
    node = GeomNode('obj')
    node.addGeom(geom)
    star = NodePath('obj')
    star.attachNewNode(node)
    star.setColor(Vec4(1,0,0,1))
    star.setTransparency(TransparencyAttrib.MAlpha)
    star.reparentTo(base.render)

    # # build grid space on table range.
    grids = []
    for x in range(300,500,100):
        for y in range(-400,-200,100):
            grids.append([x,y,-55])
    print len(grids)
    print grids
    gdb = db.GraspDB()
    tps.saveToDB(grids, gdb)


    base.run()