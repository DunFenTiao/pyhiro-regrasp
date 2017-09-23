# plot the shortest path using hrp5robot

from robotsim.nextage import nxt
from robotsim.nextage import nxtplot
from manipulation.grip.robotiq85 import rtq85nm

from manipulation.regrasp import regriptpp_quick
from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from panda3d.core import *
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
import os
import numpy as np
from utils import dbcvt as dc



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
    #objpath = os.path.join(this_dir, "objects", "t2tube.stl")
    objpath = os.path.join(this_dir, "objects", "CameraFrontCase.stl")
    workcellpath = os.path.join(this_dir, "objects", "ipadbox.stl")
    dbobjname = dbobjname = os.path.splitext(os.path.basename(objpath))[0]

    regrip = regriptpp_quick.RegripTpp(objpath, nxtrobot, handpkg, gdb)


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

    #import time
    for idstart, start in zip(idsglist, sgrotmatlist):
        for idgoal, goal in zip(idsglist, sgrotmatlist):
            print "task:", idstart, idgoal
            if idstart == idgoal:
                midnum = 0
                pathnum = 1
            else:
                #tic = time.clock()
                regrip.findshortestpath(start, goal, idstart, idgoal, base)
                pathnum = len(regrip.directshortestpaths)
                print "pathnum", pathnum
                # toc = time.clock()
                # print "(toc - tic:regrip.findshortestpath)", (toc - tic)
                # print "pathnum", pathnum
                midnum = 0
                if pathnum == 0:
                    midnum = -1  # no respathultpath
                else:
                    path = regrip.directshortestpaths[0]
                    print path
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('mid') and i < len(path) - 1:
                            midnum = midnum + 1
                            # print midnum
                            continue
                        if pathnode.startswith('goal'):
                            break

            sql = "INSERT INTO freegrip.path_ct(startid,goalid,pathnum,shortestregrasplength) VALUES('%d','%d','%d','%d')" % \
                      (idstart, idgoal, pathnum, midnum)
            gdb.execute(sql)



    base.run()