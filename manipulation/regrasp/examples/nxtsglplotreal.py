# plot the shortest path using hrp5robot

from robotsim.nextage import nxt
from robotsim.nextage import nxtplot
from manipulation.grip.robotiq85 import rtq85nm

from manipulation.regrasp import regriptpp
from manipulation.binpicking import dropworkcell

from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from panda3d.core import *
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
import os
import numpy as np
import trimesh

def getMotionSequence(regrip, id = 0):
    """
    generate motion sequence using the shortest path
    right arm

    :param: regrip an object of the regriptpp.RegripTpp class
    :param: id the pathid

    :return: [[waist, shoulder, sixjoints],...]

    author: weiwei
    date: 20170113
    """

    if len(regrip.directshortestpaths) == 0:
        print "no path found"
        return

    pathnidlist = regrip.directshortestpaths[id]

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

    #base = pandactrl.World(camp=[300,-2000,1000], lookatp=[300,0,0])
    base = pandactrl.World(camp=[5000, -1000, 1000], lookatp=[0, 0, 0])

    # ttube.stl
    this_dir = "E:/project/manipulation/regrasp_onworkcell/dropsimulation"
    # objpath = os.path.join(this_dir, "objects", "ttube.stl")
    # workcellpath = os.path.join(this_dir, "objects", "workcell22.stl")

    objpath = os.path.join(this_dir, "objects", "t2tube.stl")
    #objpath = os.path.join(this_dir, "objects", "planerearstay.stl")
    #objpath = os.path.join(this_dir, "objects", "CameraFrontCase.stl")

    workcellpath = os.path.join(this_dir, "objects", "ipadbox.stl")
    #objpath = os.path.join(this_dir, "objects", "boxobject.stl")
    #workcellpath = os.path.join(this_dir, "objects", "camerafrontcase_workcell.stl")

    regrip = regriptpp.RegripTpp(objpath,nxtrobot, handpkg, gdb)

    #camera affordance
    # placement2 = Mat4(1.0, 1.51952974647e-06, 1.51952929173e-06, 0.0, 1.51952974647e-06, -2.85851513127e-07, -1.0, 0.0,
    #                   -1.51952929173e-06, 1.0, -2.85853843707e-07, 0.0, \
    #                   400 + 2.78598856926,-300 - 0.546460807323, 24.8261165619 - 55, 1.0)
    # placement3 = Mat4(0.999522387981, -0.0218000356108, 0.0219041351229, 0.0, -0.0218000356108, 0.00500142620876,
    #                   0.999749839306, 0.0, -0.0219041351229, -0.999749839306, 0.00452379556373, 0.0, \
    #                   400 + 2.8217818737, -300 - 0.479823321104, 11.6208639145 - 55, 1.0)

    #startrotmat4 = placement3
    #goalrotmat4 = placement2

    #t2tube
    rplacement1 = Mat4(1.0, -0.000112805166282, 0.000112921290565, 0.0, -0.000112805166282, 0.00102891668212,
                       0.999999463558, 0.0, -0.000112921290565, -0.999999463558, 0.00102890387643, 0.0, \
                       400 - 47.7748527527, -100 + 3.9369931221, -0.00943910703063 - 55, 1.0)

    lplacement5 = Mat4(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, \
                       300 - 47.7774200439, 200 - 18.7865543365, 15.0 - 55+5, 1.0)

    startrotmat4 = rplacement1
    goalrotmat4 = lplacement5

    # one time show for start and end
    objstart = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    objstart.setMat(startrotmat4)
    objend = pg.genObjmnp(objpath, color=Vec4(.3, .3, 0, .5))
    objend.setMat(goalrotmat4)

    objstart.reparentTo(base.render)
    objend.reparentTo(base.render)
    pg.plotAxisSelf(base.render)

    # show workcell
    worktrimesh = trimesh.load_mesh(workcellpath)
    geom1 = pg.packpandageom(worktrimesh.vertices,
                             worktrimesh.face_normals,
                             worktrimesh.faces)
    node = GeomNode('obj')
    node.addGeom(geom1)
    workcell = NodePath('obj')
    workcell.attachNewNode(node)
    workcell.setPos(0, 0, 0)
    workcell.reparentTo(base.render)

    #show table
    this_dir = "E:/project/manipulation/regrasp_onworkcell/examples"
    ttpath = Filename.fromOsSpecific(
        os.path.join(os.path.split(this_dir)[0] + os.sep, "grip", "supports", "tabletop.egg"))
    ttnodepath = NodePath("tabletop")
    ttnodepath.setPos(0, 0, -55)
    ttl = loader.loadModel(ttpath)
    ttl.instanceTo(ttnodepath)
    ttnodepath.reparentTo(base.render)

    import time
    tic = time.clock()
    regrip.findshortestpath(startrotmat4, goalrotmat4, base)
    toc = time.clock()
    print "time",toc - tic
    print "path result", len(regrip.directshortestpaths)

    id=0
    pltfig = plt.figure()
    regrip.plotgraph(pltfig)
    regrip.plotshortestpath(pltfig, id=id)
    plt.axis("equal")
    plt.show()

    # set execute
    import robotcon.nextage as nxtcon
    import robotcon.rtq85 as rtq85con

    nxts = nxtcon.NxtSocket()
    rtq85rs = rtq85con.Rtq85Socket(handname='rgt')
    rtq85ls = rtq85con.Rtq85Socket(handname='lft')
    rtq85rs.initialize()
    rtq85ls.initialize()
    rtq85rs.openhandto(regrip.robothand.jawwidthopen)
    rtq85ls.openhandto(regrip.robothand.jawwidthopen)
    nxts.initialize()

    #values
    [objms, numikrms, jawwidth] = getMotionSequence(regrip, id)
    nxtmnp = [None]
    objmnp = [None]
    counter = [0]

    def updateshow(objms, numikrms, jawwidth, nxtmnp, objmnp, counter, nxtrobot, handpkg, objpath, task):
        if counter[0] < len (numikrms):
            if nxtmnp[0] is not None:
                nxtmnp[0].detachNode()
            if objmnp[0] is not None:
                objmnp[0].detachNode()
            print counter[0]
            print numikrms[counter[0]]
            nxtrobot.movearmfkr(numikrms[counter[0]])

            #move robot
            alljnts = [numikrms[counter[0]][0], 0, 0]
            alljnts.extend([i for i in numikrms[counter[0]][1]])
            alljnts.extend([i for i in nxtrobot.initjnts[9:]])
            nxtrobot.movealljnts(alljnts)

            nxtmnp[0] = nxtplot.genmnp(nxtrobot, handpkg, jawwidthrgt=jawwidth[counter[0]])
            nxtrobot.goinitpose()
            nxtmnp[0].reparentTo(base.render)
            objmnp[0] = pg.genObjmnp(objpath, color = Vec4(.7,.7,0,1))
            objmnp[0].setMat(objms[counter[0]])
            objmnp[0].reparentTo(base.render)

            if counter[0] >= 1:
                if base.inputmgr.keyMap['space'] is False:
                    pass
                else:
                    c2e = counter[0] - 1
                    alljnts = [numikrms[c2e][0], 0, 0]
                    alljnts.extend([i for i in numikrms[c2e][1]])
                    alljnts.extend([i for i in nxtrobot.initjnts[9:]])
                    nxts.movejnts15(alljnts)
                    if jawwidth[c2e] < 85:
                        rtq85rs.openhandto(0)
                    else:
                        rtq85rs.openhandto(85)
                    counter[0] += 1
                    base.inputmgr.keyMap['space'] = False
            else:
                counter[0] += 1
        else:
            if base.inputmgr.keyMap['space'] is True:
                c2e = counter[0] - 1
                alljnts = [numikrms[c2e][0], 0, 0]
                alljnts.extend([i for i in numikrms[c2e][1]])
                alljnts.extend([i for i in nxtrobot.initjnts[9:]])
                nxts.movejnts15(alljnts)
                if jawwidth[c2e] < 85:
                    rtq85rs.openhandto(0)
                else:
                    rtq85rs.openhandto(85)
                nxts.movejnts15(nxtrobot.initjnts)

                return task.done
        return task.again
    taskMgr.doMethodLater(1, updateshow, "updateshow",
                          extraArgs = [objms, numikrms, jawwidth, nxtmnp, objmnp, counter, nxtrobot, handpkg, objpath],
                          appendTask = True)



    base.run()