#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from manipulation.binpicking import dropworkcell
from manipulation.regrasp_onworkcell.grip.nextage import nxt
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
from database import dbaccess as db
import time



class Dropworkcellgrip(object):
    """
    manipulation.Dropworkcellgrip  take into account
    the position and orientation of the object
    in position and rotation around z axis
    """

    def __init__(self, objpath,objpathWorkcell, handpkg, dbidstablepos,readser):
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        self.objtrimesh=None

        self.dbidstablepos = dbidstablepos

        [objrotmat, objposmat]=self.loadDropStablePos()
        self.loadObjModel(objpath,objrotmat,objposmat)

        self.objcom = self.objtrimesh.center_mass
        self.objtrimeshconv=self.objtrimesh.convex_hull
        # oc means object convex
        self.ocfacets, self.ocfacetnormals = self.objtrimeshconv.facets_over(.9999)

        # for dbaccess

        # use two bulletworld, one for the ray, the other for the tabletop
        self.bulletworldray = BulletWorld()
        self.bulletworldhp = BulletWorld()
        # plane to remove hand
        self.planebullnode = cd.genCollisionPlane(offset=-55)
        self.bulletworldhp.attachRigidBody(self.planebullnode)


        #workcell to remove hand

        self.workcellmesh = trimesh.load_mesh(objpathWorkcell)
        self.objgeom = pandageom.packpandageom(self.workcellmesh.vertices, self.workcellmesh.face_normals,
                                               self.workcellmesh.faces)
        self.objmeshbullnode = cd.genCollisionMeshGeom(self.objgeom)
        self.bulletworldhp.attachRigidBody(self.objmeshbullnode)
        node = GeomNode('obj')
        node.addGeom(self.objgeom)
        self.worcell = NodePath('obj')
        self.worcell.attachNewNode(node)
        self.worcell.reparentTo(base.render)


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
        #get dropfreegrip

        self.loadDropfreeGrip()

    def loadIDObj(self):
        sql = "SELECT idobject FROM object WHERE name LIKE '%s'" % self.dbobjname
        returnlist = gdb.execute(sql)
        if len(returnlist) != 0:
            idobject = returnlist[0][0]
        else:
            sql = "INSERT INTO object(name) VALUES('%s')" % self.dbobjname
            idobject = gdb.execute(sql)
        return idobject


    def loadObjModel(self, ompath, objrotmat, objposmat):
        #print "loadObjModel(self, ompath,objrotmat,objposmat):", objrotmat, objposmat

        self.objtrimesh = trimesh.load_mesh(ompath)

        # add pos and rot to origin trimensh

        self.objtrimesh.vertices = np.transpose(np.dot(objrotmat, np.transpose(self.objtrimesh.vertices)))
        self.objtrimesh.vertices = self.objtrimesh.vertices + objposmat

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
        # "success load drop stable pos"

        objrotmat = dropstableposdata[1][self.dbidstablepos-1]
        objposmat = dropstableposdata[2][self.dbidstablepos-1]
        return [objrotmat,objposmat]

    def loadDropfreeGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: jiayao
        date: 20170811
        """

        freeairgripdata = self.gdb.loadDropFreeGrip(self.dbobjname, handname = self.handname,idstablepos=self.dbidstablepos)
        if freeairgripdata is None:
            print("no freeairgrip")
            return None
            #raise ValueError("Plan the freeairgrip first!")
        else:
            self.freegripid = freeairgripdata[0]
            self.freegripcontacts = freeairgripdata[1]
            self.freegripnormals = freeairgripdata[2]
            self.freegriprotmats = freeairgripdata[3]
            self.freegripjawwidth = freeairgripdata[4]
            self.freeairgripid = freeairgripdata[5]
            return 1


    def gentpsgrip(self, base):
        """
        Originally the code of this function is embedded in the removebadfacet function
        It is separated on 20170608 to enable common usage of placements for different hands

        :return:

        author: weiwei
        date: 20170608
        """

        self.tpsgripcontacts = []
        self.tpsgripnormals = []
        self.tpsgriprotmats = []
        self.tpsgripjawwidth = []
        # the id of the grip in freeair
        self.tpsgripidfreeair = []
        self.tpsgripiddropfree = []


        for j, rotmat in enumerate(self.freegriprotmats):
            tpsgriprotmat = rotmat
            # check if the hand collide with tabletop
            # tmprtq85 = self.rtq85hnd
            tmphnd = self.hand
            # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
            initmat = tmphnd.getMat()
            initjawwidth = tmphnd.jawwidth
            # open the hand to ensure it doesnt collide with surrounding obstacles
            # tmprtq85.setJawwidth(self.freegripjawwidth[j])
            tmphnd.setJawwidth(80)
            tmphnd.setMat(tpsgriprotmat)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworldhp.contactTest(hndbullnode)
            # print result.getNumContacts()
            if not result.getNumContacts():
                self.tpsgriprotmats.append(tpsgriprotmat)
                cct0 = self.freegripcontacts[j][0]
                cct1 = self.freegripcontacts[j][1]
                self.tpsgripcontacts.append([cct0, cct1])
                cctn0 = self.freegripnormals[j][0]
                cctn1 = self.freegripnormals[j][1]
                self.tpsgripnormals.append([cctn0, cctn1])
                self.tpsgripjawwidth.append(self.freegripjawwidth[j])
                self.tpsgripidfreeair.append(self.freeairgripid[j])
                self.tpsgripiddropfree.append(self.freegripid[j])

            tmphnd.setMat(initmat)
            tmphnd.setJawwidth(initjawwidth)



    def saveToDB(self):
        """
        save dropworkcellgrip
        dropworkcellgrip take the position and orientation of stable object on th eworkcell into account ,


        :param discretesize:
        :param gdb:
        :return:

        author: jiayao
        date: 20170816
        """


        # save freetabletopgrip
        idhand = gdb.loadIdHand(self.handname)
        idobject = gdb.loadIdObject(self.dbobjname)


        for i in range(len(self.tpsgripcontacts)):
            #print self.freeairgripid[i]
            sql = "INSERT INTO freegrip.dropworkcellgrip(idobject, contactpnt0, contactpnt1, \
                                       contactnormal0, contactnormal1, rotmat, jawwidth, idhand,iddropstablepos,iddropfreegrip,idfreeairgrip) \
                                      VALUES('%s', '%s', '%s', '%s', '%s', '%s', '%s', %d, %d,%d,%d)" % \
                  (idobject, dc.v3ToStr(self.tpsgripcontacts[i][0]), dc.v3ToStr(self.tpsgripcontacts[i][1]),
                   dc.v3ToStr(self.tpsgripnormals[i][0]), dc.v3ToStr(self.tpsgripnormals[i][1]),
                   dc.mat4ToStr(self.tpsgriprotmats[i]), str(self.tpsgripjawwidth[i]), idhand, self.dbidstablepos,self.tpsgripiddropfree[i], \
                   self.tpsgripidfreeair[i])
            gdb.execute(sql)
        print "save ok"


    def removebadfacetsshow(self, base, doverh=.1):
        """
        remove the facets that cannot support stable placements

        :param: doverh: d is the distance of mproj to supportfacet boundary, h is the height of com
                when fh>dmg, the object tends to fall over. setting doverh to 0.033 means
                when f>0.1mg, the object is judged to be unstable
        :return:

        author: weiwei
        date: 20161213
        """

        plotoffsetfp = 10
        # print self.counter

        if self.counter < len(self.ocfacets):
            i = self.counter
        # for i in range(len(self.ocfacets)):
            geom = pg.packpandageom(self.objtrimeshconv.vertices,
                                           self.objtrimeshconv.face_normals[self.ocfacets[i]],
                                           self.objtrimeshconv.faces[self.ocfacets[i]])
            geombullnode = cd.genCollisionMeshGeom(geom)
            self.bulletworldray.attachRigidBody(geombullnode)
            pFrom = Point3(self.objcom[0], self.objcom[1], self.objcom[2])
            pTo = self.objcom+self.ocfacetnormals[i]*99999
            pTo = Point3(pTo[0], pTo[1], pTo[2])
            result = self.bulletworldray.rayTestClosest(pFrom, pTo)
            self.bulletworldray.removeRigidBody(geombullnode)
            if result.hasHit():
                hitpos = result.getHitPos()
                pg.plotArrow(base.render, spos=self.objcom,
                             epos = self.objcom+self.ocfacetnormals[i], length=100)

                facetinterpnt = np.array([hitpos[0],hitpos[1],hitpos[2]])
                facetnormal = np.array(self.ocfacetnormals[i])
                bdverts3d, bdverts2d, facetmat4 = pg.facetboundary(self.objtrimeshconv, self.ocfacets[i],
                                                                     facetinterpnt, facetnormal)
                for j in range(len(bdverts3d)-1):
                    spos = bdverts3d[j]
                    epos = bdverts3d[j+1]
                    pg.plotStick(base.render, spos, epos, thickness = 1, rgba=[.5,.5,.5,1])

                facetp = Polygon(bdverts2d)
                facetinterpnt2d = rm.transformmat4(facetmat4, facetinterpnt)[:2]
                apntpnt = Point(facetinterpnt2d[0], facetinterpnt2d[1])
                dist2p = apntpnt.distance(facetp.exterior)
                dist2c = np.linalg.norm(np.array([hitpos[0],hitpos[1],hitpos[2]])-np.array([pFrom[0],pFrom[1],pFrom[2]]))
                if dist2p/dist2c < doverh:
                    print "not stable"

                else:
                    pol_ext = LinearRing(bdverts2d)
                    d = pol_ext.project(apntpnt)
                    p = pol_ext.interpolate(d)
                    closest_point_coords = list(p.coords)[0]
                    closep = np.array([closest_point_coords[0], closest_point_coords[1], 0])
                    closep3d = rm.transformmat4(rm.homoinverse(facetmat4), closep)[:3]
                    pg.plotDumbbell(base.render, spos=facetinterpnt, epos=closep3d, thickness=1.5, rgba=[0,0,1,1])

                    for j in range(len(bdverts3d)-1):
                        spos = bdverts3d[j]
                        epos = bdverts3d[j+1]
                        pg.plotStick(base.render, spos, epos, thickness = 1.5, rgba=[0,1,0,1])

                    # geomoff = pg.packpandageom(self.objtrimeshconv.vertices +
                    #                                np.tile(plotoffsetfp * self.ocfacetnormals[i],
                    #                                        [self.objtrimeshconv.vertices.shape[0], 1]),
                    #                         self.objtrimeshconv.face_normals[self.ocfacets[i]],
                    #                         self.objtrimeshconv.faces[self.ocfacets[i]])
                    #
                    # nodeoff = GeomNode('supportfacet')
                    # nodeoff.addGeom(geomoff)
                    # staroff = NodePath('supportfacet')
                    # staroff.attachNewNode(nodeoff)
                    # staroff.setColor(Vec4(1,0,1,1))
                    # staroff.setTransparency(TransparencyAttrib.MAlpha)
                    # staroff.setTwoSided(True)
                    # staroff.reparentTo(base.render)
            self.counter+=1
        else:
            self.counter=0


    def grpshow(self, base,grip):

        sql = "SELECT dropworkcellgrip.rotmat, dropworkcellgrip.jawwidth FROM dropworkcellgrip WHERE \
                                           dropworkcellgrip.iddropstablepos=%d\
                                            AND dropworkcellgrip.iddropworkcellgrip=%d" % (self.dbidstablepos,grip)
        result = self.gdb.execute(sql)
        for resultrow in result:
            hndrotmat = dc.strToMat4(resultrow[0])
            hndjawwidth = float(resultrow[1])
            # show grasps
            #tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[0, 1, 0, .1])
            tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[0, 1, 0, 0.7])
            tmprtq85.setMat(hndrotmat)
            tmprtq85.setJawwidth(hndjawwidth)
            # tmprtq85.setJawwidth(80)
            tmprtq85.reparentTo(base.render)

    def grpsshow(self, base):

        sql = "SELECT dropworkcellgrip.rotmat, dropworkcellgrip.jawwidth FROM dropworkcellgrip WHERE \
                                           dropworkcellgrip.iddropstablepos=%d" % self.dbidstablepos
        result = self.gdb.execute(sql)
        for resultrow in result:
            hndrotmat = dc.strToMat4(resultrow[0])
            hndjawwidth = float(resultrow[1])
            # show grasps
            #tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[0, 1, 0, .1])
            tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[0, 1, 0, 0.7])
            tmprtq85.setMat(hndrotmat)
            tmprtq85.setJawwidth(hndjawwidth)
            # tmprtq85.setJawwidth(80)
            tmprtq85.reparentTo(base.render)

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





def loadidstablepos():
    gdb = db.GraspDB()
    stablepos = []
    sql = "SELECT freegrip.dropstablepos.iddropstablepos FROM freegrip.dropstablepos"
    data = gdb.execute(sql)
    if len(data) != 0:
        for i in range(len(data)):
            stablepos.append(int(data[i-1][0]))
        return stablepos
    else:
        print "drop workcell to have stable pose first!"

def drawonetime(self):
        # change dbidstablepos to draw one stablepos and its grips
        dbidstablepos = 1
        tps = Dropworkcellgrip(objpath, objpathWorkcell, handpkg, dbidstablepos, gdb)
        if tps.loadDropfreeGrip():
            # draw grip
            tps.grpshow(base)
            # draw stablepos
            tps.showOnePlacementAndAssociatedGrips(base)
            tps = None
        else:
            print "no drop free grip"


def freegripRotMove(objname,handname):
    """

    :param objname:
    :return:
        for each dropstablepos
        caculate its grips after rot and move
        and save to database dropfreegrip to remove the hand around workcell next
        mention that the dropstablepos rot,pos
        rotmat=(np.transpose(rot),pos)
    """
    gdb = db.GraspDB()

    idhand = gdb.loadIdHand(handname)
    idobject=gdb.loadIdObject(objname)

    sql = "SELECT dropstablepos.iddropstablepos\
                               FROM dropstablepos, object \
                          WHERE dropstablepos.idobject = object.idobject AND object.name like '%s'" % (objname)

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
        #rotanglelist = [float(x) for x in result1[:, 2]]
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

            rotmat4=pg.npToMat4(np.transpose(pg.mat3ToNp(rotangle)), ttoppos)

            ttpcct0 = rotmat4.xformPoint(cct0)
            ttpcct1 = rotmat4.xformPoint(cct1)
            ttpcctn0 = rotmat4.xformVec(cctn0)
            ttpcctn1 = rotmat4.xformVec(cctn1)
            ttpgriprotmat = freegriprotmat * rotmat4
            #ttpgriprotmat =  rotmat4*freegriprotmat

            sql = "INSERT INTO dropfreegrip(contactpnt0, contactpnt1, contactnormal0, contactnormal1, \
                                          rotmat, jawwidth, idfreeairgrip, iddropstablepos,idhand,idobject) VALUES \
                                          ('%s', '%s', '%s', '%s', '%s', '%s', %d, %d,%d,%d) " % \
                  (dc.v3ToStr(ttpcct0), dc.v3ToStr(ttpcct1), dc.v3ToStr(ttpcctn0), dc.v3ToStr(ttpcctn1), \
                   dc.mat4ToStr(ttpgriprotmat), str(jawwidth), idfreegrip, idtabletopplacements, idhand, idobject)
            gdb.execute(sql)




def updateDBwithIK(gdb, robot, objname,armname='rgt'):
        """

        :param gdb:
        :param robot:
        :param armname: the name of the arm used rgt or lft
        :param rethandx: the distance of retract along handx direction, default 50mm
        :param retworldz: the distance of retract along worldz direction, default 50mm
        :param retworlda: the distance of retract along assembly/approaching direction in the world, default 50mm
        :return:

        author: jiayao
        date: 20170820
        """

        # load idarm
        idarm = gdb.loadIdArm(armname)

        # load retraction distances
        rethandx, retworldz, retworlda, worldz = gdb.loadIKRet()
        # worlda is default for the general grasps on table top
        # for assembly at start and goal, worlda is computed by assembly planner
        worlda = Vec3(0, 0, 1)

        # select idrobot
        idrobot = gdb.loadIdRobot(robot)

        feasibility = {}
        feasibility_handx = {}
        feasibility_handxworldz = {}
        feasibility_worlda = {}
        feasibility_worldaworldz = {}

        sql = "SELECT freegrip.dropworkcellgrip.iddropworkcellgrip,freegrip.dropworkcellgrip.contactpnt0, freegrip.dropworkcellgrip.contactpnt1, \
                        freegrip.dropworkcellgrip.rotmat FROM freegrip.dropworkcellgrip,dropstablepos, object WHERE \
                        dropworkcellgrip.iddropstablepos = dropstablepos.iddropstablepos AND \
                           freegrip.dropworkcellgrip.idobject = object.idobject AND object.name LIKE '%s'" % objname
        result = gdb.execute(sql)
        if len(result) == 0:
            raise ValueError("Plan the dropworkcellgrip first!")
        idcounter = 0
        tic = time.clock()
        for resultrow in result:
            print idcounter * 1.0 / len(result)
            idcounter += 1
            toc = time.clock()
            print toc - tic
            ttgsid = int(resultrow[0])
            ttgscct0 = dc.strToV3(resultrow[1])
            ttgscct1 = dc.strToV3(resultrow[2])
            ttgsrotmat = dc.strToMat4(resultrow[3])
            ttgsfgrcenter = (ttgscct0 + ttgscct1) / 2
            handx = ttgsrotmat.getRow3(0)
            ttgsfgrcenterhandx = ttgsfgrcenter + handx * rethandx
            ttgsfgrcenterhandxworldz = ttgsfgrcenterhandx + worldz * retworldz
            ttgsfgrcenterworlda = ttgsfgrcenter + worlda * retworlda
            ttgsfgrcenterworldaworldz = ttgsfgrcenterworlda + worldz * retworldz

            ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
            ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
            ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
            ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
            ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
            ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())

            if robot.numik(ttgsfgrcenternp, ttgsrotmat3np, armname) is not None:
                feasibility[ttgsid] = 'True'
            else:
                feasibility[ttgsid] = 'False'
            if robot.numik(ttgsfgrcenternp_handx, ttgsrotmat3np, armname) is not None:
                feasibility_handx[ttgsid] = 'True'
            else:
                feasibility_handx[ttgsid] = 'False'
            if robot.numik(ttgsfgrcenternp_handxworldz, ttgsrotmat3np, armname) is not None:
                feasibility_handxworldz[ttgsid] = 'True'
            else:
                feasibility_handxworldz[ttgsid] = 'False'
            if robot.numik(ttgsfgrcenternp_worlda, ttgsrotmat3np, armname) is not None:
                feasibility_worlda[ttgsid] = 'True'
            else:
                feasibility_worlda[ttgsid] = 'False'
            if robot.numik(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np, armname) is not None:
                feasibility_worldaworldz[ttgsid] = 'True'
            else:
                feasibility_worldaworldz[ttgsid] = 'False'

            # insert ik table
            # sql = "INSERT IGNORE INTO ik(idrobot, idarm, idtabletopgrips, feasibility, feasibility_handx, feasibility_handxworldz, \
            #         feasibility_worlda, feasibility_worldaworldz) VALUES (%d, %d, %d, '%s', '%s', '%s', '%s', '%s')" % \
            #         (idrobot, idarm, ttgsid, feasibility[ttgsid], feasibility_handx[ttgsid], feasibility_handxworldz[ttgsid], \
            #          feasibility_worlda[ttgsid], feasibility_worldaworldz[ttgsid])
            sql = "INSERT IGNORE INTO ik_drop(idrobot, idarm, iddropworkcellgrip, feasibility, feasibility_handx, feasibility_handxworldz, \
                                feasibility_worlda, feasibility_worldaworldz) VALUES (%d, %d, %d, '%s', '%s', '%s', '%s', '%s')" % \
                  (idrobot, idarm, ttgsid, feasibility[ttgsid], feasibility_handx[ttgsid],
                   feasibility_handxworldz[ttgsid], \
                   feasibility_worlda[ttgsid], feasibility_worldaworldz[ttgsid])
            gdb.execute(sql)


if __name__ == '__main__':
    base = pandactrl.World(camp=[5000, -1000, 1000], lookatp=[0, 0, 0])
    #this_dir, this_filename = os.path.split(__file__)
    this_dir = "E:/project/manipulation/regrasp_onworkcell/dropsimulation"
    #objpath = os.path.join(this_dir, "objects", "ttube.stl")
    #objpath = os.path.join(this_dir, "objects", "planefrontstay.stl")
    #objpathWorkcell = os.path.join(this_dir, "objects", "workcell22.stl")

    #objpath = os.path.join(this_dir, "objects", "t2tube.stl")
    #objpath = os.path.join(this_dir, "objects", "planerearstay.stl")
    objpath = os.path.join(this_dir, "objects", "CameraFrontCase.stl")
    #objpathWorkcell = os.path.join(this_dir, "objects", "ipadbox.stl")
    #objpathWorkcell = os.path.join(this_dir, "objects", "workcellmix.stl")


    #objpath = os.path.join(this_dir, "objects", "boxobject.stl")
    #objpathWorkcell = os.path.join(this_dir, "objects", "boxobject_workcell2.stl")
    objpathWorkcell = os.path.join(this_dir, "objects", "camerafrontcase_workcell.stl")


    objname = os.path.splitext(os.path.basename(objpath))[0]
    print objpathWorkcell
    handpkg = rtq85nm
    gdb = db.GraspDB()
    #

    #move free grip to stable pos place
    handname=handpkg.getHandName()
    freegripRotMove(objname,handname)
    #select stablepos from database
    stablepos=loadidstablepos()

    #caculate the grips
    #stablepos=[4]
    for dbidstablepos in stablepos:
        tps = Dropworkcellgrip(objpath, objpathWorkcell, handpkg, dbidstablepos, gdb)
        if tps.loadDropfreeGrip():
            tps.gentpsgrip(base)
            tps.saveToDB()
            tps.showOnePlacementAndAssociatedGrips(base)
            tps = None
        else:
            print "no drop free grip"

    # #get ik of grips
    nxtrobot = nxt.NxtRobot()

    updateDBwithIK(gdb, nxtrobot,objname,armname="rgt", )

    #draw one time

    nxtrobot = nxt.NxtRobot()

    # plot nxtrobot
    from manipulation.regrasp_onworkcell.grip.nextage import nxt
    from manipulation.regrasp_onworkcell.grip.nextage import nxtplot
    from manipulation.regrasp_onworkcell.grip.robotiq85 import rtq85nm

    nxtrobot = nxt.NxtRobot()
    nxtrobot.goinitpose()

    handpkg = rtq85nm
        # handpkg = hrp5threenm
        # nxtrobot.movewaist(-15)
    nxtmnp = nxtplot.genmnp(nxtrobot, handpkg)
    nxtmnp.reparentTo(base.render)

    base.run()