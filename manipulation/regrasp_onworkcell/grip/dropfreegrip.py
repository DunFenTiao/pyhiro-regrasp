#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from manipulation.binpicking import dropworkcell
from manipulation.grip.nextage import nxt
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
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
from manipulation.grip import freegripcontactpairs as fgcp
from manipulation.grip import freegrip as fgcp
from database import dbaccess as db



class DropFreegrip(object):
    """
    manipulation.DropFreegrip  take into account
    the position and orientation of the object
    in position and rotation around z axis
    """

    def __init__(self, objpath,handpkg):
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]
        self.objtrimesh=None


        #[objrotmat, objposmat]=self.loadDropStablePos()
        #self.loadObjModel(objpath,objrotmat,objposmat)
        #self.loadFreeAirGrip()

        # for dbaccess

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.1])
        # self.rtq85hnd = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])

        # for dbsave
        # each tpsmat4 corresponds to a set of tpsgripcontacts/tpsgripnormals/tpsgripjawwidth list
        self.tpsmat4s = None
        self.tpsgripcontacts = None
        self.tpsgripnormals = None
        self.tpsgripjawwidth = None

        # for ocFacetShow
        self.counter = 0

        self.gdb = gdb
        #self.loadDropfreeGrip()

    def loadDropStablePos(self):
        """
        load self.dropid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: jiayao
        date: 20170810
        """
        self.gdb=db.GraspDB()
        dropstableposdata = self.gdb.loadDropStablePos(self.dbobjname)
        if dropstableposdata is None:
            raise ValueError("Plan the drop stable pos first!")
        print "success load drop stable pos"

        #objrotmat = dropstableposdata[1][self.dbidstablepos-1]
        #objposmat = dropstableposdata[2][self.dbidstablepos-1]


        #return [objrotmat,objposmat]


    def loadDropfreeGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: jiayao
        date: 20170811
        """

        dropfreegripdata = self.gdb.loadDropFreeGrip(self.dbobjname, handname = self.handname,idstablepos=self.dbidstablepos)
        if  dropfreegripdata is None:

            print("no  dropfreegrip grip")
            return None
            #raise ValueError("Plan the freeairgrip first!")
        else:
            self.freegripid =  dropfreegripdata[0]
            self.freegripcontacts =  dropfreegripdata[1]
            self.freegripnormals =  dropfreegripdata[2]
            self.freegriprotmats =  dropfreegripdata[3]
            self.freegripjawwidth =  dropfreegripdata[4]
            return 1

    def saveToDB(self):
        """
        save  dropfreegripdatagrip
        dropworkcellgrip take the position and orientation of stable object on th eworkcell into account ,


        :param discretesize:
        :param gdb:
        :return:

        author: jiayao
        date: 20170816
        """


        # save freetabletopgrip
        idhand = gdb.loadIdHand(self.handname)

        idobject = 1

        for i in range(len(self.gripcontacts)):
            sql = "INSERT INTO freegrip.dropfreegrip(idobject, contactpnt0, contactpnt1, \
                               contactnormal0, contactnormal1, rotmat, jawwidth, idhand,idstablepos) \
                              VALUES('%s', '%s', '%s', '%s', '%s', '%s', '%s', %d, %d)" % \
                  (idobject, dc.v3ToStr(self.gripcontacts[i][0]), dc.v3ToStr(self.gripcontacts[i][1]),
                   dc.v3ToStr(self.gripcontactnormals[i][0]), dc.v3ToStr(self.gripcontactnormals[i][1]),
                   dc.mat4ToStr(self.griprotmats[i]), str(self.gripjawwidth[i]), idhand, self.idstablepos)
            gdb.execute(sql)
        print "save ok"



    def showOnePlacementAndAssociatedGrips(self, base):
        """
        show one placement and its associated grasps
        :param base:
        :return:
        """
        geom = pg.packpandageom(self.objtrimesh.vertices,
                                self.objtrimesh.face_normals,
                                self.objtrimesh.faces)
        node = GeomNode('obj')
        node.addGeom(geom)
        star = NodePath('obj')
        star.attachNewNode(node)
        star.setColor(Vec4(.7, 0.3, 0, 1))
        star.setTransparency(TransparencyAttrib.MAlpha)
        #star.setMat(objrotmat)
        star.reparentTo(base.render)
        # for j in range(len(self.tpsgriprotmats)):
        #     # for j in range(13,14):
        #     hndrotmat = self.tpsgriprotmats[j]
        #     hndjawwidth = self.tpsgripjawwidth[j]
        #     # show grasps
        #     tmphnd = self.handpkg.newHandNM(hndcolor=[0, 1, 0, .5])
        #     tmphnd.setMat(hndrotmat)
        #     tmphnd.setJawwidth(hndjawwidth)
        #     tmphnd.reparentTo(base.render)

    def freegripRotMove(self):
        # self.freegripid = freeairgripdata[0]
        # self.freegripcontacts = freeairgripdata[1]
        # self.freegripnormals = freeairgripdata[2]
        # self.freegriprotmats = freeairgripdata[3]
        # self.freegripjawwidth = freeairgripdata[4]

        idhand = 1
        idobject = 1

        gdb = db.GraspDB()
        sql = "SELECT dropstablepos.iddropstablepos\
                               FROM dropstablepos, object \
                          WHERE dropstablepos.idobject = object.idobject AND object.name like '%s'" % (self.dbobjname)

        result = gdb.execute(sql)
        print result
        if len(result) == 0:
            print "no DropStablePos select"
            return None

        for idfree in result:
            idfree = int(idfree[0])
            sql = "SELECT dropstablepos.iddropstablepos, \
                                      dropstablepos.pos, dropstablepos.rot,\
                                      freeairgrip.contactpnt0, freeairgrip.contactpnt1, \
                                      freeairgrip.contactnormal0, freeairgrip.contactnormal1, \
                                      freeairgrip.rotmat, freeairgrip.jawwidth, freeairgrip.idfreeairgrip \
                                      FROM dropstablepos,freeairgrip WHERE \
                                          freeairgrip.idhand = %d AND \
                                          dropstablepos.iddropstablepos = %d" % (idhand, idfree)
            result1 = gdb.execute(sql)
            if len(result1) == 0:
                print "no free air grasp availalbe"
                continue
            if len(result1) > 20000:
                result1 = result1[0::int(len(result1) / 20000.0)]
            result1 = np.asarray(result1)
            idtabletopplacementslist = [int(x) for x in result1[:, 0]]
            tabletoppositionlist = [dc.strToV3(x) for x in result1[:, 1]]
            # rotanglelist = [float(x) for x in result1[:, 2]]
            rotanglelist = [dc.strToMat3(x) for x in result1[:, 2]]
            freegripcontactpoint0list = [dc.strToV3(x) for x in result1[:, 3]]
            freegripcontactpoint1list = [dc.strToV3(x) for x in result1[:, 4]]
            freegripcontactnormal0list = [dc.strToV3(x) for x in result1[:, 5]]
            freegripcontactnormal1list = [dc.strToV3(x) for x in result1[:, 6]]
            freegriprotmatlist = [dc.strToMat4(x) for x in result1[:, 7]]
            freegripjawwidthlist = [float(x) for x in result1[:, 8]]
            freegripidlist = [int(x) for x in result1[:, 9]]
            for idtabletopplacements, ttoppos, rotangle, cct0, cct1, cctn0, cctn1, \
                freegriprotmat, jawwidth, idfreegrip in zip(idtabletopplacementslist, \
                                                            tabletoppositionlist, rotanglelist, \
                                                            freegripcontactpoint0list, freegripcontactpoint1list, \
                                                            freegripcontactnormal0list, freegripcontactnormal1list,
                                                            freegriprotmatlist, freegripjawwidthlist, \
                                                            freegripidlist):
                # rotmat = rm.rodrigues([0, 0, 1], rotangle)
                rotmat = rotangle
                rotmat4 = Mat4(rotmat[0][0], rotmat[0][1], rotmat[0][2], 0,
                               rotmat[1][0], rotmat[1][1], rotmat[1][2], 0,
                               rotmat[2][0], rotmat[2][1], rotmat[2][2], 0,
                               ttoppos[0], ttoppos[1], ttoppos[2], 1)
                # rotmat4 = pg.cvtMat4(rotmat, ttoppos)
                # rotmat4=pg.cvtMat4(rotangle, ttoppos)
                ttpcct0 = rotmat4.xformPoint(cct0)
                ttpcct1 = rotmat4.xformPoint(cct1)
                ttpcctn0 = rotmat4.xformVec(cctn0)
                ttpcctn1 = rotmat4.xformVec(cctn1)
                ttpgriprotmat = freegriprotmat * rotmat4
                sql = "INSERT INTO dropfreegrip(contactpnt0, contactpnt1, contactnormal0, contactnormal1, \
                                          rotmat, jawwidth, idfreeairgrip, iddropstablepos,idhand,idobject) VALUES \
                                          ('%s', '%s', '%s', '%s', '%s', '%s', %d, %d,%d,%d) " % \
                      (dc.v3ToStr(ttpcct0), dc.v3ToStr(ttpcct1), dc.v3ToStr(ttpcctn0), dc.v3ToStr(ttpcctn1), \
                       dc.mat4ToStr(ttpgriprotmat), str(jawwidth), idfreegrip, idtabletopplacements, idhand, idobject)
                gdb.execute(sql)


    def loadFreeAirGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: weiwei
        date: 20170110
        """

        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname, handname = self.handname)
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripid = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]






def loadidstablepos():
    gdb = db.GraspDB()
    stablepos = []
    sql = "SELECT freegrip.dropstablepos.idstablepos FROM freegrip.dropstablepos"
    data = gdb.execute(sql)
    if len(data) != 0:
        for i in range(len(data)):
            stablepos.append(int(data[i][0]))
        return stablepos
    else:
        print "drop workcell to have stable pose first!"



if __name__ == '__main__':
    base = pandactrl.World(camp=[5000, -1000, 1000], lookatp=[0, 0, 0])
    this_dir, this_filename = os.path.split(__file__)
    this_dir = "E:/project/manipulation/regrasp_onworkcell/dropsimulation"
    objpath = os.path.join(this_dir, "objects", "ttube.stl")
    objpathWorkcell = os.path.join(this_dir, "objects", "workcell22.stl")
    print objpathWorkcell
    handpkg = rtq85nm

    #select stablepos from database
    #stablepos=loadidstablepos()
    gdb = db.GraspDB()
    #for each stable pos caculate the freegrip and workcell grip
    #for dbidstablepos in stablepos:
        #print dbidstablepos
        #drop free grip
    dropfreegriptst = DropFreegrip(objpath, handpkg)
    dropfreegriptst.freegripRotMove()
        #dropfreegriptst.saveToDB(gdb)


    # nxtrobot = nxt.NxtRobot()
    # gdb = db.GraspDB()
    #
    #
    # # plot nxtrobot
    # from manipulation.grip.nextage import nxt
    # from manipulation.grip.nextage import nxtik
    # from manipulation.grip.nextage import nxtplot
    # from manipulation.grip.robotiq85 import rtq85nm
    #
    # nxtrobot = nxt.NxtRobot()
    # nxtrobot.goinitpose()
    #
    # handpkg = rtq85nm
    #     # handpkg = hrp5threenm
    #     # nxtrobot.movewaist(-15)
    # nxtmnp = nxtplot.genmnp(nxtrobot, handpkg)
    # nxtmnp.reparentTo(base.render)

    base.run()