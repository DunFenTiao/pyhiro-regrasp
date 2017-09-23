# plot the shortest path using hrp5robot

from robotsim.nextage import nxt
from robotsim.nextage import nxtplot
from manipulation.grip.robotiq85 import rtq85nm

from manipulation.regrasp_onworkcell import regriptpp_fixturequick
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

if __name__=='__main__':
    gdb = db.GraspDB()
    nxtrobot = nxt.NxtRobot()
    handpkg = rtq85nm

    #base = pandactrl.World(camp=[300,-2000,1000], lookatp=[300,0,0])
    base = pandactrl.World(camp=[5000, -1000, 1000], lookatp=[0, 0, 0])

    # ttube.stl
    this_dir = "E:/project/manipulation/regrasp_onworkcell/dropsimulation"
    #objpath = os.path.join(this_dir, "objects", "t2tube.stl")
    #objpath = os.path.join(this_dir, "objects", "t2tube.stl")
    objpath = os.path.join(this_dir, "objects", "CameraFrontCase.stl")
    #workcellpath = os.path.join(this_dir, "objects", "ipadbox.stl")
    workcellpath= os.path.join(this_dir, "objects", "camerafrontcase_workcell.stl")
    dbobjname= os.path.splitext(os.path.basename(objpath))[0]
    regrip = regriptpp_fixturequick.RegripTpp(objpath, workcellpath,nxtrobot, handpkg, gdb)

    #start and goal
    #table top placements
    #regrip = regriptpp_fixturequick.RegripTpp(objpath, workcellpath, nxtrobot, handpkg, gdb)

    sql = "SELECT startgoal.idstartgoal,startgoal.rotmat,startgoal.idfreetabletopplacement FROM startgoal,freetabletopplacement,object WHERE \
                    startgoal.idfreetabletopplacement=freetabletopplacement.idfreetabletopplacement AND \
                     freetabletopplacement.idobject=object.idobject AND object.name LIKE '%s'" % dbobjname
    result = gdb.execute(sql)
    result = np.asarray(result)
    idsglist = [int(x) for x in result[:, 0]]
    sgrotmatlist = [dc.strToMat4(x) for x in result[:, 1]]
    idfreeplacementlist=[int(x) for x in result[:, 2]]

    import time
    for idstart, start in zip(idsglist, sgrotmatlist):
        for idgoal, goal in zip(idsglist, sgrotmatlist):
            #if idstart==42 and idgoal==96:
            print idstart, idgoal
            if idstart == idgoal:
                midnum = 0
                pathnum = 1
            else:
                tic=time.clock()
                regrip.findshortestpath(start, goal, idstart, idgoal, base)
                toc=time.clock()
                print "time:",toc-tic
                pathnum = len(regrip.directshortestpaths)
                print "pathnum", pathnum
                midnum = 0
                if pathnum == 0:
                    midnum = -1
                else:
                    path = regrip.directshortestpaths[0]
                    print path
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('mid') and i < len(path) - 1:
                            midnum = midnum + 1

                            continue
                        if pathnode.startswith('goal'):
                            break
                    print "midnum", midnum
            sql = "INSERT INTO freegrip.path_cc2(startid,goalid,pathnum,shortestregrasplength) VALUES('%d','%d','%d','%d')" % (
            idstart, idgoal, pathnum, midnum)
            gdb.execute(sql)

    base.run()