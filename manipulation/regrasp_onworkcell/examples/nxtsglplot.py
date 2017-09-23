# plot the shortest path using hrp5robot

from robotsim.nextage import nxt
from robotsim.nextage import nxtplot
from manipulation.grip.robotiq85 import rtq85nm

from manipulation.regrasp_onworkcell import regriptpp
from manipulation.binpicking import dropworkcell

from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from panda3d.core import *
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
import os
import numpy as np
import trimesh
from utils import dbcvt as dc

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
            print "two node path"
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

            print numikrlist
        else:
            if i == 0:
                print "not two nodepath, starting node, transfer"
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

                #print "grpposworldaworldz, grprot", grpposworldaworldz, grprot
                #print "armjntsgrpworldaworldz,numikr", armjntsgrpworldaworldz

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
                    print "not two node path, middle nodes, if transit, pass"
                    # not two node path, middle nodes, if transit, pass
                    if regrip.regg.edge[pathnidlist[i]][pathnidlist[i+1]]['edgetype'] == "transit":
                        print "transit pass"
                        pass
                    else:
                        print "# not two node path, middle nodes, if transfer"
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
                    print "not two node path, end nodes, transfer"
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
    #
    workcellpath = os.path.join(this_dir, "objects", "ipadbox.stl")

    #objpath = os.path.join(this_dir, "objects", "boxobject.stl")
    #workcellpath = os.path.join(this_dir, "objects", "camerafrontcase_workcell.stl")



    regrip = regriptpp.RegripTpp(objpath, workcellpath,nxtrobot, handpkg, gdb)

    #t2tube start goal from drop stable pos :ok
    # startrotmat4=Mat4(0.955147504807,-0.293643295765,-0.0383004359901,0,\
    #                   -0.0423758253455,-0.00752697139978,-0.999073505402,0,\
    #                   0.293082922697,0.955885529518,-0.0196327473968,0, \
    #                   584.945800781, 19.0754261017, 13.8659801483,1.0 )
    #
    # goalrotmat4 = Mat4(0.681344091892,-0.355974793434,0.639571845531,0,\
    #                    0.682250499725,-0.00766822695732,-0.731078207493,0,\
    #                    0.265149831772,0.934464037418,0.237639322877,0, \
    #                    604.364807129, -44.871257782, 13.5711507797,1.0 )

    # startrotmat4=Mat4(0.955147504807, -0.293643295765, -0.0383004359901,0,\
    #                   -0.0423758253455, -0.00752697139978, -0.999073505402, 0,\
    #         0.293082922697, 0.955885529518, -0.0196327473968,0, \
    #                   584.945800781, 19.0754261017, 13.8659801483,1)

    #(500,200)left

    # lplacement1=Mat4(1.0,-0.000112805166282,0.000112921290565,0.0,-0.000112805166282,0.00102891668212,0.999999463558,0.0,-0.000112921290565,-0.999999463558,0.00102890387643,0.0,\
    #                 400-47.7748527527,200+3.9369931221,-0.00943910703063-55,1.0)
    # lplacement2=Mat4(0.918899595737,-0.307511538267,-0.247103631496,0.0,-0.307511538267,-0.166003227234,-0.936952292919,0.0,0.247103631496,0.936952292919,-0.247103646398,0.0,\
    #                 300-39.1018600464,300+14.1088991165,61.6203193665-55,1.0)
    # lplacement3 = Mat4(6.12323426293e-17,0.0,1.0,0.0,0.0,1.0,0.0,0.0,-1.0,0.0,6.12323426293e-17,0.0,\
    #                   300+3.95093536377,200-18.7865543365,44.9999961853-55,1.0)
    # lplacement4 = Mat4(6.12323426293e-17,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,6.12323426293e-17,0.0,\
    #                   300-3.95093536377,200-18.7865543365,135.0-55,1.0)
    # lplacement5 = Mat4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,\
    #                   300-47.7774200439,200-18.7865543365,15.0-55,1.0)
    # lplacement6 = Mat4(0.0315224789083,0.968476176262,0.247103646398,0.0,0.968476176262,0.0315251536667,-0.247103303671,0.0,-0.247103646398,0.247103303671,-0.936952352524,0.0,\
    #                   300-18.7241039276,300-47.8398323059,29.0131397247-55,1.0)
    # # (500,-200)right
    # rplacement1 = Mat4(1.0, -0.000112805166282, 0.000112921290565, 0.0, -0.000112805166282, 0.00102891668212,
    #                   0.999999463558, 0.0, -0.000112921290565, -0.999999463558, 0.00102890387643, 0.0, \
    #                   400 - 47.7748527527, 300 + 3.9369931221, -0.00943910703063 - 55, 1.0)
    # rplacement2 = Mat4(0.918899595737, -0.307511538267, -0.247103631496, 0.0, -0.307511538267, -0.166003227234,
    #                   -0.936952292919, 0.0, 0.247103631496, 0.936952292919, -0.247103646398, 0.0, \
    #                   500 - 39.1018600464, -200 + 14.1088991165, 61.6203193665 - 55, 1.0)
    # rplacement3 = Mat4(6.12323426293e-17, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 6.12323426293e-17, 0.0, \
    #                   500 + 3.95093536377, -200 - 18.7865543365, 44.9999961853 - 55, 1.0)
    # rplacement4 = Mat4(6.12323426293e-17, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 6.12323426293e-17, 0.0, \
    #                   500 - 3.95093536377, -200 - 18.7865543365, 135.0 - 55, 1.0)
    # rplacement5 = Mat4(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, \
    #                   500 - 47.7774200439, -200 - 18.7865543365, 15.0 - 55, 1.0)
    # rplacement6 = Mat4(0.0315224789083, 0.968476176262, 0.247103646398, 0.0, 0.968476176262, 0.0315251536667,\
    #                   -0.247103303671, 0.0, -0.247103646398, 0.247103303671, -0.936952352524, 0.0, \
    #                   300 - 18.7241039276, -300 - 47.8398323059, 29.0131397247 - 55, 1.0)
    #
    #
    # # (500,-200)right
    # rplacement1 = Mat4(1.0, -0.000112805166282, 0.000112921290565, 0.0, -0.000112805166282, 0.00102891668212,
    #                    0.999999463558, 0.0, -0.000112921290565, -0.999999463558, 0.00102890387643, 0.0, \
    #                    400 - 47.7748527527, -100 + 3.9369931221, -0.00943910703063 - 55+5, 1.0)
    #
    # lplacement5 = Mat4(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, \
    #                    300 - 47.7774200439, 200 - 18.7865543365, 15.0 - 55 + 5, 1.0)
    #
    # startrotmat4= rplacement1
    # goalrotmat4 = lplacement5



    # # placement1 = Mat4(-0.319523245096, -0.00040422056918, 0.947578370571, 0.0, -0.00040422056918, 0.999999880791,
    # #                   0.000290279567707, 0.0, -0.947578370571, -0.000290279567707, -0.319523364305, 0.0, \
    # #                   500 - 30.2711086273, 200 - 0.0485660322011, 156.955215454 - 55, 1.0)
    # # placement2 = Mat4(0.409777253866, 0.771226286888, -0.487126916647, 0.0, 0.771226286888, -0.00773812970147,
    # #                   0.63651406765, 0.0, 0.487126916647, -0.63651406765, -0.597960889339, 0.0, \
    # #                   500 - 47.1169128418, 200 + 106.373542786, -13.255355835 - 55, 1.0)
    # # placement3 = Mat4(0.997545421124, -0.000393244525185, 0.0700214803219, 0.0, -0.000393244525185, 0.99993699789,
    # #                   0.0112179713324, 0.0, -0.0700214803219, -0.0112179713324, 0.997482419014, 0.0, \
    # #                   500 - 129.007247925, 200 + 0.0773421004415, 11.9952945709 - 55, 1.0)
    # # placement4 = Mat4(0.409778445959, -0.771227538586, -0.487123966217, 0.0, -0.771227538586, -0.00774340564385,
    # #                   -0.636512517929, 0.0, 0.487123966217, 0.636512517929, -0.597964942455, 0.0, \
    # #                   500 - 47.1169815063, 200 + -106.373687744, -13.2552099228 - 55, 1.0)
    #
    # #object-affordance
    # placement2 = Mat4(1.0, 1.51952974647e-06, 1.51952929173e-06, 0.0, 1.51952974647e-06, -2.85851513127e-07, -1.0, 0.0,
    #                   -1.51952929173e-06, 1.0, -2.85853843707e-07, 0.0, \
    #                   400 + 2.78598856926,-300 - 0.546460807323, 24.8261165619 - 55, 1.0)
    # placement3 = Mat4(0.999522387981, -0.0218000356108, 0.0219041351229, 0.0, -0.0218000356108, 0.00500142620876,
    #                   0.999749839306, 0.0, -0.0219041351229, -0.999749839306, 0.00452379556373, 0.0, \
    #                   400 + 2.8217818737, -300 - 0.479823321104, 11.6208639145 - 55, 1.0)
    # #ipadbox
    # # placement2 = Mat4(1.0, 1.51952974647e-06, 1.51952929173e-06, 0.0, 1.51952974647e-06, -2.85851513127e-07, -1.0, 0.0,
    # #                   -1.51952929173e-06, 1.0, -2.85853843707e-07, 0.0, \
    # #                   400 + 2.78598856926, -300 - 0.546460807323, 24.8261165619 - 55, 1.0)
    # # placement3 = Mat4(0.999522387981, -0.0218000356108, 0.0219041351229, 0.0, -0.0218000356108, 0.00500142620876,
    # #                   0.999749839306, 0.0, -0.0219041351229, -0.999749839306, 0.00452379556373, 0.0, \
    # #                   400 + 2.8217818737, -300 - 0.479823321104, 11.6208639145 - 55, 1.0)
    #
    # startrotmat4 = placement3
    # goalrotmat4 = placement2


    # one time show for start and end
    # objstart = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    # objstart.setMat(startrotmat4)
    # objend = pg.genObjmnp(objpath, color=Vec4(.3, .3, 0, .5))
    # objend.setMat(goalrotmat4)
    #
    # objstart.reparentTo(base.render)
    # objend.reparentTo(base.render)
    # pg.plotAxisSelf(base.render)

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



    # show table
    # this_dir, this_filename = os.path.split(__file__)
    # ttpath = Filename.fromOsSpecific(
    #     os.path.join(os.path.split(this_dir)[0] + os.sep, "grip", "supports", "tabletop.egg"))
    # ttnodepath = NodePath("tabletop")
    # ttnodepath.setPos(0, 0, -55)
    # ttl = loader.loadModel(ttpath)
    # ttl.instanceTo(ttnodepath)
    # ttnodepath.reparentTo(base.render)
    #
    dbobjname  = os.path.splitext(os.path.basename(objpath))[0]
    sql = "SELECT startgoal.idstartgoal,startgoal.rotmat,startgoal.idfreetabletopplacement FROM startgoal,freetabletopplacement,object WHERE \
                        startgoal.idfreetabletopplacement=freetabletopplacement.idfreetabletopplacement AND \
                         freetabletopplacement.idobject=object.idobject AND object.name LIKE '%s'" % dbobjname
    result = gdb.execute(sql)
    result = np.asarray(result)
    idsglist = [int(x) for x in result[:, 0]]
    sgrotmatlist = [dc.strToMat4(x) for x in result[:, 1]]
    idfreeplacementlist = [int(x) for x in result[:, 2]]

    for idstart, start, idfreepls in zip(idsglist, sgrotmatlist, idfreeplacementlist):
        for idgoal, goal, idfreeplg in zip(idsglist, sgrotmatlist, idfreeplacementlist):
            #print idstart, idgoal
            if idstart == 2 and idgoal == 1:
                print start, goal
                startrotmat4 = start
                goalrotmat4 = goal
                regrip.findshortestpath(start, goal, base)
                break
    #
    objstart = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    objstart.setMat(startrotmat4)
    objend = pg.genObjmnp(objpath, color=Vec4(.3, .3, 0, .5))
    objend.setMat(goalrotmat4)

    objstart.reparentTo(base.render)
    objend.reparentTo(base.render)
    #regrip.findshortestpath(startrotmat4, goalrotmat4, base)

    pltfig = plt.figure()
    regrip.plotgraph(pltfig)


    #
    print "path result",len(regrip.directshortestpaths)

    # for id in range(len(regrip.directshortestpaths)):
    #     print "id",idzhen
    #     regrip.plotshortestpath(pltfig, id)
    #     plt.axis("equal")
    #     plt.show()
    #     [objms, numikrms, jawwidth] = getMotionSequence(regrip, id)
    #     print "objms, numikrms, jawwidth",objms, numikrms, jawwidth

    id = 0

    regrip.plotshortestpath(pltfig, id)
    plt.axis("equal")
    plt.show()
    [objms, numikrms, jawwidth] = getMotionSequence(regrip, id)

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
        else:
            print "task done initial"

        return task.again


    taskMgr.doMethodLater(1, updateshow, "updateshow",
                          extraArgs = [objms, numikrms, jawwidth, nxtmnp, objmnp, counter, nxtrobot, handpkg, objpath],
                          appendTask = True)



    base.run()