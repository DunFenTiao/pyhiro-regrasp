# plot the shortest path using hrp5robot

from robotsim.nextage import nxt
from robotsim.nextage import nxtplot
from manipulation.grip.robotiq85 import rtq85nm

from manipulation.regrasp import regriptpp
from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from panda3d.core import *
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
import os
import numpy as np
from utils import dbcvt as dc

def getMotionSequence(regrip):
    """
    generate motion sequence using the shortest path
    right arm

    :param: regrip an object of the regriptpp.RegripTpp class
    :param: gdb GraspDB object

    :return: [[waist, shoulder, sixjoints],...]

    author: weiwei
    date: 20170113
    """

    if len(regrip.directshortestpaths) == 0:
        print "no path found"
        return

    pathnidlist = regrip.directshortestpaths[0]
    numikrlist = []
    objmat4list = []
    jawwidth = []
    for i in range(len(pathnidlist) - 1):
        if i == 0 and len(pathnidlist) == 2:
            # two node path
            ## starting node
            nid = pathnidlist[i]
            grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
            grppos = regrip.regg.node[nid]['fgrcenter']
            grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
            grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
            grprot = regrip.regg.node[nid]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid]['jawwidth']
            armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot)
            armjntsgrp = regrip.robot.numikr(grppos, grprot)
            armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot)
            armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot)
            numikrlist.append(armjntsgrphandx)
            jawwidth.append(regrip.robothand.jawwidthopen)
            numikrlist.append(armjntsgrp)
            jawwidth.append(regrip.robothand.jawwidthopen)
            numikrlist.append(armjntsgrp)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrpworlda)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrpworldaworldz)
            jawwidth.append(grpjawwidth)
            objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
            objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
            objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
            objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4worldaworldz)
            ## first node
            nid = pathnidlist[i + 1]
            grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
            grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
            grppos = regrip.regg.node[nid]['fgrcenter']
            grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
            grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
            grprot = regrip.regg.node[nid]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid]['jawwidth']
            armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot)
            armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot)
            armjntsgrp = regrip.robot.numikr(grppos, grprot)
            armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot)
            armjntsgrphandxworldz = regrip.robot.numikr(grpposhandxworldz, grprot)
            numikrlist.append(armjntsgrpworldaworldz)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrpworlda)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrp)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrp)
            jawwidth.append(regrip.robothand.jawwidthopen)
            numikrlist.append(armjntsgrphandx)
            jawwidth.append(regrip.robothand.jawwidthopen)
            numikrlist.append(armjntsgrphandxworldz)
            jawwidth.append(regrip.robothand.jawwidthopen)
            objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
            objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
            objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
            objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
            objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
            objmat4list.append(objmat4worldaworldz)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4handxworldz)
        else:
            if i == 0:
                # not two nodepath, starting node, transfer
                ## starting node
                nid = pathnidlist[i]
                grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                grppos = regrip.regg.node[nid]['fgrcenter']
                grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                grprot = regrip.regg.node[nid]['hndrotmat3np']
                grpjawwidth = regrip.regg.node[nid]['jawwidth']
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot)
                armjntsgrp = regrip.robot.numikr(grppos, grprot)
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot)
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot)
                numikrlist.append(armjntsgrphandx)
                jawwidth.append(regrip.robothand.jawwidthopen)
                numikrlist.append(armjntsgrp)
                jawwidth.append(regrip.robothand.jawwidthopen)
                numikrlist.append(armjntsgrp)
                jawwidth.append(grpjawwidth)
                numikrlist.append(armjntsgrpworlda)
                jawwidth.append(grpjawwidth)
                numikrlist.append(armjntsgrpworldaworldz)
                jawwidth.append(grpjawwidth)
                objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                objmat4list.append(objmat4handx)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4worlda)
                objmat4list.append(objmat4worldaworldz)
                ## first node
                nid = pathnidlist[i + 1]
                grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                grppos = regrip.regg.node[nid]['fgrcenter']
                grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                grprot = regrip.regg.node[nid]['hndrotmat3np']
                grpjawwidth = regrip.regg.node[nid]['jawwidth']
                armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot)
                armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot)
                armjntsgrp = regrip.robot.numik(grppos, grprot)
                armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot)
                armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot)
                numikrlist.append([0, armjntsgrpworldaworldz])
                jawwidth.append(grpjawwidth)
                numikrlist.append([0, armjntsgrpworlda])
                jawwidth.append(grpjawwidth)
                numikrlist.append([0, armjntsgrp])
                jawwidth.append(grpjawwidth)
                numikrlist.append([0, armjntsgrp])
                jawwidth.append(regrip.robothand.jawwidthopen)
                numikrlist.append([0, armjntsgrphandx])
                jawwidth.append(regrip.robothand.jawwidthopen)
                numikrlist.append([0, armjntsgrphandxworldz])
                jawwidth.append(regrip.robothand.jawwidthopen)
                objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                objmat4list.append(objmat4worldaworldz)
                objmat4list.append(objmat4worlda)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4handx)
                objmat4list.append(objmat4handxworldz)
            else:
                if i + 1 != len(pathnidlist) - 1:
                    # not two node path, middle nodes, if transit, pass
                    if regrip.regg.edge[pathnidlist[i]][pathnidlist[i+1]]['edgetype'] == "transit":
                        pass
                    else:
                        # not two node path, middle nodes, if transfer
                        ## middle first
                        nid = pathnidlist[i]
                        grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                        grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                        grppos = regrip.regg.node[nid]['fgrcenter']
                        grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                        grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                        grprot = regrip.regg.node[nid]['hndrotmat3np']
                        grpjawwidth = regrip.regg.node[nid]['jawwidth']
                        armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot)
                        armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot)
                        armjntsgrp = regrip.robot.numik(grppos, grprot)
                        armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot)
                        armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot)
                        numikrlist.append([0, armjntsgrphandxworldz])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrphandx])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrp])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrp])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrpworlda])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrpworldaworldz])
                        jawwidth.append(grpjawwidth)
                        objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                        objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                        objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                        objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                        objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                        objmat4list.append(objmat4handxworldz)
                        objmat4list.append(objmat4handx)
                        objmat4list.append(objmat4)
                        objmat4list.append(objmat4)
                        objmat4list.append(objmat4worlda)
                        objmat4list.append(objmat4worldaworldz)
                        ## middle second
                        nid = pathnidlist[i + 1]
                        grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                        grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                        grppos = regrip.regg.node[nid]['fgrcenter']
                        grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                        grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                        grprot = regrip.regg.node[nid]['hndrotmat3np']
                        grpjawwidth = regrip.regg.node[nid]['jawwidth']
                        armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot)
                        armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot)
                        armjntsgrp = regrip.robot.numik(grppos, grprot)
                        armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot)
                        armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot)
                        numikrlist.append([0, armjntsgrpworldaworldz])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrpworlda])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrp])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrp])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrphandx])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrphandxworldz])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                        objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                        objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                        objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                        objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                        objmat4list.append(objmat4worldaworldz)
                        objmat4list.append(objmat4worlda)
                        objmat4list.append(objmat4)
                        objmat4list.append(objmat4)
                        objmat4list.append(objmat4handx)
                        objmat4list.append(objmat4handxworldz)
                else:
                    # not two node path, end nodes, transfer
                    ## second to last node
                    nid = pathnidlist[i]
                    grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                    grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                    grppos = regrip.regg.node[nid]['fgrcenter']
                    grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                    grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                    grprot = regrip.regg.node[nid]['hndrotmat3np']
                    grpjawwidth = regrip.regg.node[nid]['jawwidth']
                    armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot)
                    armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot)
                    armjntsgrp = regrip.robot.numik(grppos, grprot)
                    armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot)
                    armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot)
                    numikrlist.append([0, armjntsgrphandxworldz])
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append([0, armjntsgrphandx])
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append([0, armjntsgrp])
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append([0, armjntsgrp])
                    jawwidth.append(grpjawwidth)
                    numikrlist.append([0, armjntsgrpworlda])
                    jawwidth.append(grpjawwidth)
                    numikrlist.append([0, armjntsgrpworldaworldz])
                    jawwidth.append(grpjawwidth)
                    objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                    objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                    objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                    objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                    objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                    objmat4list.append(objmat4handxworldz)
                    objmat4list.append(objmat4handx)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4worlda)
                    objmat4list.append(objmat4worldaworldz)
                    ## last node
                    nid = pathnidlist[i + 1]
                    grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                    grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                    grppos = regrip.regg.node[nid]['fgrcenter']
                    grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                    grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                    grprot = regrip.regg.node[nid]['hndrotmat3np']
                    grpjawwidth = regrip.regg.node[nid]['jawwidth']
                    armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot)
                    armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot)
                    armjntsgrp = regrip.robot.numikr(grppos, grprot)
                    armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot)
                    armjntsgrphandxworldz = regrip.robot.numikr(grpposhandxworldz, grprot)
                    numikrlist.append(armjntsgrpworldaworldz)
                    jawwidth.append(grpjawwidth)
                    numikrlist.append(armjntsgrpworlda)
                    jawwidth.append(grpjawwidth)
                    numikrlist.append(armjntsgrp)
                    jawwidth.append(grpjawwidth)
                    numikrlist.append(armjntsgrp)
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append(armjntsgrphandx)
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append(armjntsgrphandxworldz)
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                    objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                    objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                    objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                    objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                    objmat4list.append(objmat4worldaworldz)
                    objmat4list.append(objmat4worlda)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4handx)
                    objmat4list.append(objmat4handxworldz)
    return [objmat4list, numikrlist, jawwidth]

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
    #objpath = os.path.join(os.path.split(os.path.split(this_dir)[0])[0], "grip", "objects", "planefrontstay.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planerearstay.stl")

    this_dir = "E:/project/manipulation/regrasp_onworkcell/dropsimulation"
    objpath = os.path.join(this_dir, "objects", "t2tube.stl")
    #objpath = os.path.join(this_dir, "objects", "planerearstay.stl")
    #objpath = os.path.join(this_dir, "objects", "brick.stl")
    #objpath = os.path.join(this_dir, "objects", "CameraFrontCase.stl")

    #objpath = os.path.join(this_dir, "objects", "boxobject.stl")


    regrip = regriptpp.RegripTpp(objpath, nxtrobot, handpkg, gdb)

    # rplacement1 = Mat4(1.0, -0.000112805166282, 0.000112921290565, 0.0, -0.000112805166282, 0.00102891668212,
    #                    0.999999463558, 0.0, -0.000112921290565, -0.999999463558, 0.00102890387643, 0.0, \
    #                    400 - 47.7748527527, -100 + 3.9369931221, -0.00943910703063 - 55 + 5, 1.0)
    #
    # lplacement5 = Mat4(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, \
    #                    300 - 47.7774200439, 200 - 18.7865543365, 15.0 - 55 + 5, 1.0)
    #
    # startrotmat4 = rplacement1
    # goalrotmat4 = lplacement5

    # objstart = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    # objstart.setMat(startrotmat4)
    # objend = pg.genObjmnp(objpath, color=Vec4(.3, .3, 0, .5))
    # objend.setMat(goalrotmat4)
    #
    # objstart.reparentTo(base.render)
    # objend.reparentTo(base.render)

    # #
    # # # import time
    # # # tic = time.clock()
    #regrip.findshortestpath(startrotmat4, goalrotmat4, base)
    dbobjname = os.path.splitext(os.path.basename(objpath))[0]
    sql = "SELECT startgoal.idstartgoal,startgoal.rotmat,startgoal.idfreetabletopplacement FROM startgoal,freetabletopplacement,object WHERE \
                           startgoal.idfreetabletopplacement=freetabletopplacement.idfreetabletopplacement AND \
                            freetabletopplacement.idobject=object.idobject AND object.name LIKE '%s'" % dbobjname
    result = gdb.execute(sql)
    result = np.asarray(result)
    idsglist = [int(x) for x in result[:, 0]]
    sgrotmatlist = [dc.strToMat4(x) for x in result[:, 1]]
    idfreeplacementlist = [int(x) for x in result[:, 2]]

    for idstart, start in zip(idsglist, sgrotmatlist):
        for idgoal, goal in zip(idsglist, sgrotmatlist):
            # print idstart, idgoal
            if idstart == 2 and idgoal == 1:
                print start, goal
                startrotmat4=start
                goalrotmat4=goal
                regrip.findshortestpath(start, goal, base)
                break
                #
    objstart = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    objstart.setMat(startrotmat4)
    objend = pg.genObjmnp(objpath, color=Vec4(.3, .3, 0, .5))
    objend.setMat(goalrotmat4)

    objstart.reparentTo(base.render)
    objend.reparentTo(base.render)


    print "path result on table", len(regrip.directshortestpaths)
    # # # # toc = time.clock()
    # # # # print len(regrip.directshortestpaths[0])
    # # # # print toc-tic
    # # # # assert False
    # # # #
    pltfig = plt.figure()
    regrip.plotgraph(pltfig)
    regrip.plotshortestpath(pltfig)
    plt.axis("equal")
    plt.show()
    #
    [objms, numikrms, jawwidth] = getMotionSequence(regrip)
    nxtmnp = [None]
    objmnp = [None]
    counter = [0]
    def updateshow(objms, numikrms, jawwidth, nxtmnp, objmnp, counter, nxtrobot, handpkg, objpath, task):
        if counter[0] < len(numikrms):
            if nxtmnp[0] is not None:
                nxtmnp[0].detachNode()
            if objmnp[0] is not None:
                objmnp[0].detachNode()
            print counter[0]
            print numikrms[counter[0]]
            nxtrobot.movearmfkr(numikrms[counter[0]])
            nxtmnp[0] = nxtplot.genmnp(nxtrobot, handpkg, jawwidthrgt=jawwidth[counter[0]])
            nxtrobot.goinitpose()
            nxtmnp[0].reparentTo(base.render)
            objmnp[0] = pg.genObjmnp(objpath, color = Vec4(.7,.7,0,1))
            objmnp[0].setMat(objms[counter[0]])
            # pg.plotAxisSelf(base.render,objms[counter[0]].getRow3(3), objms[counter[0]])
            objmnp[0].reparentTo(base.render)
            counter[0] += 1
        return task.again
    taskMgr.doMethodLater(1, updateshow, "updateshow",
                          extraArgs = [objms, numikrms, jawwidth, nxtmnp, objmnp, counter, nxtrobot, handpkg, objpath],
                          appendTask = True)


    base.run()