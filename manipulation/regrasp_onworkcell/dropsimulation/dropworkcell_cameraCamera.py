#!/usr/bin/python
import os
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
import trimesh
from panda3d.core import *
from panda3d.ode import *
import random
import time
from utils import dbcvt as dc
from database import dbaccess as db
import numpy as np

class ODESim(object):
    #simulation object drop on the workcell
    # set the position of object in function:addobject


    def __init__(self, base, objpath1,objpath2, nobj = 1,ntimes=1):
        self.base = base
        self.nobj = nobj
        self.ntimes= ntimes
        self.poscount=1
        self.objnodepath=None
        self.workcell = None

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath1))[0]
        print self.dbobjname

        self.smiley = trimesh.load_mesh(objpath1)
        self.smileyCount = 0
        self.lasttime0=0
        self.lasttime0rot = 0
        self.laststablepos=0

        self.setupODE()
        self.addGround()
        self.count = 0

        self.stableFlag=0

        self.partlist = []

        self.worktrimesh = trimesh.load_mesh(objpath2)
        self.addworkstl()

        self.simcontrolstart = time.time()
        self.interval0=time.time()
        self.simdrop=time.time()
        self.checkcount=0

        self.body= None
        self.preObj()
        self.addObj()
        taskMgr.add(self.updateODE, "UpdateODE")



    def setupODE(self):
        self.odeworld = OdeWorld()
        self.odeworld.setGravity(0, 0, -9.81)



        self.odespace = OdeSimpleSpace()
        self.contacts = OdeJointGroup()

    def addGround(self):
        boxx = 1000.0
        boxy = 1000.0
        boxz = 1.0
        offset=-55
        pg.plotBox(self.base.render, pos = [0,0,-1.0+offset], x = boxx*2.0, y = boxy*2.0, z = boxz*2.0, rgba=None)
        pg.plotAxisSelf(self.base.render)
        groundGeom = OdePlaneGeom(self.odespace, Vec4(0, 0, 1, offset))


    def addworkstl(self):
        workx = 0
        worky = 0
        workz = 0
        geom1 = pg.packpandageom(self.worktrimesh.vertices,
                                 self.worktrimesh.face_normals,
                                self.worktrimesh.faces)
        node = GeomNode('obj')
        node.addGeom(geom1)
        self.workcell = NodePath('obj')
        self.workcell.attachNewNode(node)
        #workcell.setColor(Vec4(.7, 0.3, 0, 1))
        #workcell.setTransparency(TransparencyAttrib.MAlpha)
        self.workcell.setPos(-workx, worky, workz)
        self.workcell.reparentTo(base.render)

        self.workcellbody = OdeBody(self.odeworld)
        mass = OdeMass()
        #mass.setSphereTotal(50000, 1)
        mass.setSphereTotal(5000,1)
        self.workcellbody.setMass(mass)
        self.workcellbody.setPosition(self.workcell.getPos())
        self.workcellbody.setQuaternion(self.workcell.getQuat())

        modelTrimesh2 = OdeTriMeshData(self.workcell, True)
        modelGeom2 = OdeTriMeshGeom(self.odespace, modelTrimesh2)
        modelGeom2.setBody(self.workcellbody)

        joint=OdeFixedJoint(self.odeworld)
        joint.attachBody(self.workcellbody,0)

        self.workcell.setPythonTag("body", self.workcellbody)
        self.partlist.append(self.workcell)


    def preObj(self):
        """

        :return: prepare add obj into odespace, set node paht ,set geom, set body mass, do only once in a loop
         author:jiayao
        time:20170811
        """
        geom = pg.packpandageom(self.smiley.vertices,
                                self.smiley.face_normals,
                                self.smiley.faces)
        node = GeomNode('obj')
        node.addGeom(geom)
        self.objnodepath = NodePath('obj')
        self.objnodepath.attachNewNode(node)
        self.objnodepath.setColor(Vec4(.7, 0.3, 0, 1))

        self.body = OdeBody(self.odeworld)
        mass = OdeMass()
        mass.setSphereTotal(5, 1)

        self.body.setMass(mass)

        self.modelTrimesh = OdeTriMeshData(self.objnodepath, True)
        self.modelGeom = OdeTriMeshGeom(self.odespace, self.modelTrimesh)

    def resetobj(self):
        #self.partlist = []
        self.contacts.empty()

        self.workcell.setPosQuat((0,0,0),Quat(1,0,0,0))
        self.workcellbody.setPosition(0,0,0)
        self.workcellbody.setQuaternion(Quat(1,0,0,0))
        modelTrimesh2 = OdeTriMeshData(self.workcell, True)
        modelGeom2 = OdeTriMeshGeom(self.odespace, modelTrimesh2)
        modelGeom2.setBody(self.workcellbody)

        joint = OdeFixedJoint(self.odeworld)
        joint.attachBody(self.workcellbody, 0)

        self.simdrop = time.time()

        self.addObj()
        self.contacts = OdeJointGroup()

    def checkrange(self,position):
        # minx = 450;
        # miny = -150;
        # minz = -150
        # maxx = 700;
        # maxy = 200;
        # maxz = 300

        #range
        # minx = 450;
        # miny = -250;
        # minz = -150
        # maxx = 700;
        # maxy = 250;
        # maxz = 300

        minx = 450;
        miny = -100;
        minz = -150
        maxx = 700;
        maxy = 100;
        maxz = 300

        if position.getZ() < minz or position.getZ() > maxz or \
                        position.getX() < minx or position.getX() > maxx or \
                        position.getY() < miny or position.getY() > maxy:
            return True
        else:
            return False

        # if body.getPosition().getZ() < minz or body.getPosition().getZ() > maxz or \
        #                 body.getPosition().getX() < minx or body.getPosition().getX() > maxx or \
        #                 body.getPosition().getY() < miny or body.getPosition().getY() > maxy:
        #     print "out of work cell range", body.getPosition()
        #     self.resetobj()
        #     task.again

    def updateODE(self, task):
        simcontrol = time.time()
        # limittime=5
        # if simcontrol - self.simcontrolstart > limittime*60:
        #     print "Time out  ",limittime,"mins"
        #     return task.done


        self.odespace.collide((self.odeworld, self.contacts), self.near_callback)
        self.odeworld.quickStep(globalClock.getDt()*1.5)

        #for smiley in self.partlist:
        body = self.objnodepath.getPythonTag("body")
        self.objnodepath.setPosQuat(body.getPosition(), Quat(body.getQuaternion()))

        self.workcellbody.setPosition(0, 0, 0)
        self.workcell.setPosQuat(self.workcellbody.getPosition(), Quat(self.workcellbody.getQuaternion()))
        joint = OdeFixedJoint(self.odeworld)
        joint.attachBody(self.workcellbody, 0)

        thistime = body.getPosition()
        thistimerot=body.getRotation()


        intervaltime = 2  # 1s
        droptime = 20  # 10s

        siminterval = time.time()

        if siminterval - self.interval0 > intervaltime:
            self.interval0 = siminterval

            if siminterval - self.simdrop > droptime:
                print "out of droptime,reset"
                self.resetobj()
                task.again

            if self.checkrange(body.getPosition()):
                print "out of work cell range",body.getPosition()
                self.resetobj()
                task.again

            diff = 1.0  # difference of pos
            if (self.checkDiff(thistime,
                                   self.lasttime0) < diff):  # and self.checkDiffRot(thistimerot, self.lasttime0):
                self.rotmat = body.getRotation()
                self.posmat = body.getPosition()
                print "get stablepos: ", self.poscount
                # save rot pos to sql database
                gdb = db.GraspDB()
                #check=self.checkDBrepeat(gdb)
                check=1
                # print "check",check
                if check:
                    self.saveToDB(gdb)
                # else:
                #     self.checkcount=self.checkcount+1
                #     if self.checkcount>=3:
                #         print "repeat 3 times,stop dropping"
                #         return task.done
                # reset the environment
                self.poscount = self.poscount + 1
                # if task num haven't all done
                if self.ntimes >= self.poscount:
                    self.resetobj()
                    task.again
                else:
                    print "Done Update"
                    return task.done
            self.lasttime0 = thistime






        self.contacts.empty()
        return task.cont



    # Collision callback
    def near_callback(self, args, geom1, geom2):
        """Callback function for the collide() method.

        This function checks if the given geoms do collide and
        creates contact joints if they do.
        """

        # Check if the objects do collide
        contacts = OdeUtil.collide(geom1, geom2)

        # Create contact joints
        world, contactgroup = args
        for cgeom in contacts.getContactGeoms():
            c = OdeContact()
            c.setGeom(cgeom)
            s = OdeSurfaceParameters()
            s.setBounce(0.0)
            s.setBounceVel(10000)
            #s.setMu(100)
            s.setMu(OdeUtil.getInfinity())
            #s.setSoftCfm(100)
            #s.setSoftErp(1)
            c.setSurface(s)
            j = OdeContactJoint(world, contactgroup, c)
            j.attach(geom1.getBody(), geom2.getBody())

    def checkDiff(self,pos1,pos2):
        """

        :return:if the two pos the pos and quration is the same.
        author: jiayao
        date: 20170817
        """
        diffpos=pos1-pos2
        #diffrot=abs(rot1-rot2)
        diff=abs(diffpos.getX())+abs(diffpos.getY())+abs(diffpos.getZ())
        # diffrot.get()
        return diff

    def checkDiffRot(self, rot1,rot2):
        """

        :return:if the two pos the pos and quration is the same.
        author: jiayao
        date: 20170817
        """
        print rot1,rot2
        #diffrot = pg.v3ToNp(rot1)-pg.v3ToNp(rot2)

        diffrot1=pg.mat3ToNp(rot1)
        print rot2
        diffrot2=pg.mat3ToNp(rot2)
        print diffrot1,diffrot2
        diffrot=diffrot1-diffrot2
        diff=(np.trace(np.dot(diffrot,diffrot)))**0.5
        print "diffrot",diff
        #haven't finished
        # diffrot.get()
        return diff


    def checkDBrepeat(self,gdb):
        """

        :return: is there repeat stable pos in database already
        not finish

        author: jiayao
        date: 20170811
        """

        dropid = []
        rot = []
        pos = []
        # access to db

        sql = "SELECT freegrip.dropstablepos.iddropstablepos,\
                           freegrip.dropstablepos.rot, \
                           freegrip.dropstablepos.pos \
                            FROM freegrip.dropstablepos, object \
                                WHERE freegrip.dropstablepos.idobject = object.idobject AND object.name like '%s'\
                                    " % (self.dbobjname)

        data = gdb.execute(sql)
        print data
        #the difference between pos
        diff=10
        diffrot=10
        if len(data) != 0:
            for i in range(len(data)):
                    print i
                    #if (self.checkDiff(self.posmat,dc.strToV3(data[i][2]))< diff) and (self.checkDiffRot(self.rotmat,dc.strToMat3(data[i][1])<diffrot)):
                    if (self.checkDiff(self.posmat, dc.strToV3(data[i][2])) < diff) :
                        print "repeat with ",int(data[i][0]),self.posmat,dc.strToV3(data[i][2])
                        return False
                    else:
                        print "no repeat"
                        return True
            #sql="DELETE FROM freegrip.dropstablepos WHERE freegrip.dropstablepos.iddropstablepos like %d" % (dropid[i])
            #data = gdb.execute(sql)

        else:
            print ("Plan the drop stable pos first!")
            return True


        print "success rank and delete repeated data"


    def saveToDB(self, gdb):
        """
        save the result to mysqldatabase

        :param gdb: is an object of the GraspDB class in the database package
        :return:

        author: jiayao
        date: 20170809
        """

        # save to database
        gdb = db.GraspDB()

        sql = "SELECT idobject FROM object WHERE name LIKE '%s'" % self.dbobjname
        returnlist = gdb.execute(sql)
        if len(returnlist) != 0:
            idobject = returnlist[0][0]

        else:
            sql = "INSERT INTO object(name) VALUES('%s')" % self.dbobjname
            idobject = gdb.execute(sql)
        print "idobject",idobject

        sql = "INSERT INTO freegrip.dropstablepos(idobject,rot,pos) VALUES('%d','%s','%s')" % \
                                     (idobject,dc.mat3ToStr(self.rotmat),dc.LVecBase3fToStr(self.posmat))
        gdb.execute(sql)

        print "Drop saved !"
        # else:
        #     print "drop stable pos already saved 1 times or duplicated filename!"



    def loadIdobject(self, handname):
        sql = "SELECT idhand FROM hand WHERE name = '%s'" % handname
        result = self.execute(sql)
        print result
        if len(result) != 0:
            idhand = int(result[0][0])
        else:
            assert "No hand found in hand table!"
        return idhand

    def addObj(self):
        """

        :return: add obj at random postion and set body, which can loop for n-times
        author:jiayao
        time:20170811
        """

        #drop random
        # axis = np.array([random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1)])
        # if (axis[0] ==0) & (axis[1] ==0) & (axis[2] ==0):
        #     axis = np.array([0.1,0.1,0.1])
        # else:
        #     axis = axis/np.linalg.norm(axis)
        # theta = random.uniform(0,360.0)
        # rotmat = Mat4.rotateMat(theta, Vec3(axis[0], axis[1], axis[2]))

        #drop straight
        axis=np.array([0,-1,0])
        theta = random.uniform(0, 360.0)
        rotmat=Mat4.rotateMat(theta, Vec3(axis[0], axis[1], axis[2]))

        self.objnodepath.setMat(rotmat)


        self.objnodepath.setPos(random.uniform(500, 680), random.uniform(-80, 80), 100.0)

        self.lasttime0 = self.objnodepath.getPos()
        self.objnodepath.reparentTo(base.render)

        self.body.setPosition(self.objnodepath.getPos())
        self.body.setQuaternion(self.objnodepath.getQuat())
        self.body.setLinearVel(0,0,0)
        self.body.setAngularVel(0,0,0)
        self.modelGeom.setBody(self.body)

        self.objnodepath.setPythonTag("body", self.body)
        self.partlist.append(self.objnodepath)



if __name__=='__main__':

    base = pandactrl.World(camp=[5000,-1000,1000], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    print this_dir

    #objpath1 = os.path.join(this_dir, "objects", "t2tube.stl")
    #objpath1 = os.path.join(this_dir, "objects", "planerearstay.stl")
    #objpath1 = os.path.join(this_dir, "objects", "planewheel.stl")
    #objpath1 = os.path.join(this_dir, "objects", "ttube.stl")
    #objpath1 = os.path.join(this_dir, "objects", "planefrontstay.stl")
    #objpath1=os.path.join(this_dir, "objects", "CameraFrontCase.stl")

    #objpath2=  os.path.join(this_dir, "objects", "workcell22.stl")
    #objpath2 = os.path.join(this_dir, "objects", "CardBoardBox.stl")
    #objpath2 = os.path.join(this_dir, "objects", "ipadbox.stl")
    #objpath2 = os.path.join(this_dir, "objects", "workcell.stl")
    #objpath2 = os.path.join(this_dir, "objects", "workcellmix.stl")
    # objpath1 = os.path.join(this_dir, "objects", "boxobject.stl")
    # objpath2 = os.path.join(this_dir, "objects", "boxobject_workcell2.stl")

    objpath1=os.path.join(this_dir, "objects", "CameraFrontCase.stl")
    objpath2=os.path.join(this_dir, "objects", "camerafrontcase_workcell.stl")

    odesim = ODESim(base, objpath1, objpath2,nobj =1,ntimes=1000)
    #plot nxtrobot
    from manipulation.regrasp_onworkcell.grip.nextage import nxt
    from manipulation.regrasp_onworkcell.grip.nextage import nxtplot
    from manipulation.regrasp_onworkcell.grip.robotiq85 import rtq85nm
    nxtrobot = nxt.NxtRobot()
    nxtrobot.goinitpose()
    handpkg = rtq85nm
    nxtmnp = nxtplot.genmnp(nxtrobot, handpkg)
    # nxtmnp.reparentTo(base.render)

    base.run()
