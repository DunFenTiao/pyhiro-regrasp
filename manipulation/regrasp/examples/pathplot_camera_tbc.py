# plot the shortest path using hrp5robot

from robotsim.nextage import nxt
from robotsim.nextage import nxtplot
from manipulation.grip.robotiq85 import rtq85nm

from manipulation.regrasp_onworkcell import regriptpp
from manipulation.binpicking import dropworkcell



import matplotlib.pyplot as plt
import matplotlib as mpl

from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from panda3d.core import *
from pandaplotutils import pandageom as pg

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
    objpath = os.path.join(this_dir, "objects", "CameraFrontCase.stl")
    workcellpath = os.path.join(this_dir, "objects", "ipadbox.stl")
    dbobjname=dbobjname = os.path.splitext(os.path.basename(objpath))[0]

    #sql = "SELECT path2.idpath2,path2.startid,path2.goalid,path2.pathnum,path2.shortestregrasplength FROM path2"
    #sql = "SELECT path2.idpath2,path2.startid,path2.goalid,path2.pathnum,path2.shortestregrasplength FROM path2"
    #sql = "SELECT pathquick2.idpathquick2,pathquick2.startid,pathquick2.goalid,pathquick2.pathnum,pathquick2.shortestregrasplength FROM pathquick2"
    # sql = "SELECT pathquick2_fixture.idpathquick2_fixture,pathquick2_fixture.startid,pathquick2_fixture.goalid,pathquick2_fixture.pathnum,pathquick2_fixture.shortestregrasplength FROM pathquick2_fixture"
    # result = gdb.execute(sql)
    # result = np.asarray(result)
    # idpath = [int(x) for x in result[:, 0]]
    # startid = [int(x) for x in result[:, 1]]
    # goalid = [int(x) for x in result[:, 2]]
    # pathnum = [int(x) for x in result[:, 3]]
    # shortestregrasplength = [int(x) for x in result[:, 4]]

    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from matplotlib.pyplot import figure, show, axes, sci
    from matplotlib import cm, colors
    from matplotlib.font_manager import FontProperties
    from numpy import amin, amax, ravel
    from numpy.random import rand


    fig = figure()
    shortestregrasplength=None
    startid=None
    goalid=None

    def drawonce(shortestregrasplength,startid,goalid):
        z = []
        for i in shortestregrasplength:
            if i == -1:
                z.append(None)
            if i == 0:
                z.append(None)
            if i!=-1 and i!=0:
                z.append(i)
        x = startid
        y = goalid
        plt.scatter(x, y, c=z, s=3, alpha=0.6, vmin=0, vmax=10,cmap=cm.jet)
        #plt.scatter(x, y, c=z, s=3, alpha=0.6, cmap=cm.jet)
        ax.set_xticks([])
        ax.set_yticks([])
        plt.colorbar()


    figtitle = 'regrasp length: camera'
    t = fig.text(0.5, 0.95, figtitle,
                 horizontalalignment='center',
                 fontproperties=FontProperties(size=16))

    ##########################
    ax=plt.subplot(511)
    ax.set_title("on table")
    # sql = "SELECT pathquick2_fixture.idpathquick2_fixture,pathquick2_fixture.startid,pathquick2_fixture.goalid,pathquick2_fixture.pathnum,pathquick2_fixture.shortestregrasplength FROM pathquick2_fixture"
    sql = "SELECT path_ct.idpath_ct,path_ct.startid,path_ct.goalid,path_ct.pathnum,path_ct.shortestregrasplength FROM path_ct"
    result = gdb.execute(sql)
    result = np.asarray(result)
    idpath = [int(x) for x in result[:, 0]]
    startid = [int(x) for x in result[:, 1]]
    goalid = [int(x) for x in result[:, 2]]
    pathnum = [int(x) for x in result[:, 3]]
    shortestregrasplength = [int(x) for x in result[:, 4]]
    drawonce(shortestregrasplength, startid, goalid)
    data1=shortestregrasplength

    ########################
    ax = plt.subplot(512)
    ax.set_title("on fixture basket time 65")
    sql = "SELECT path_cb1.idpath_cb1,path_cb1.startid,path_cb1.goalid,path_cb1.pathnum,path_cb1.shortestregrasplength FROM path_cb1"
    result = gdb.execute(sql)
    result = np.asarray(result)
    idpath = [int(x) for x in result[:, 0]]
    startid = [int(x) for x in result[:, 1]]
    goalid = [int(x) for x in result[:, 2]]
    pathnum = [int(x) for x in result[:, 3]]
    shortestregrasplength = [int(x) for x in result[:, 4]]
    drawonce(shortestregrasplength, startid, goalid)
    data2=shortestregrasplength

    #########################
    ax = plt.subplot(513)
    ax.set_title("on fixture camera affordance time 32")

    # sql = "SELECT path_.idpathquick3_fixture_quick,pathquick3_fixture_quick.startid,pathquick3_fixture_quick.goalid,pathquick3_fixture_quick.pathnum,pathquick3_fixture_quick.shortestregrasplength FROM pathquick3_fixture_quick"
    # sql = "SELECT pathquick2_fixture.idpathquick2_fixture,pathquick2_fixture.startid,pathquick2_fixture.goalid,pathquick2_fixture.pathnum,pathquick2_fixture.shortestregrasplength FROM pathquick2_fixture"
    sql = "SELECT path_cc1.idpath_cc1,path_cc1.startid,path_cc1.goalid,path_cc1.pathnum,path_cc1.shortestregrasplength FROM path_cc1"
    result = gdb.execute(sql)
    result = np.asarray(result)
    idpath = [int(x) for x in result[:, 0]]
    startid = [int(x) for x in result[:, 1]]
    goalid = [int(x) for x in result[:, 2]]
    pathnum = [int(x) for x in result[:, 3]]
    shortestregrasplength = [int(x) for x in result[:, 4]]
    drawonce(shortestregrasplength, startid, goalid)
    data3=shortestregrasplength
    # ##############################
    ax = plt.subplot(514)
    ax.set_title("on fixture camera affordance time 104")
    sql = "SELECT path_cc2.idpath_cc2,path_cc2.startid,path_cc2.goalid,path_cc2.pathnum,path_cc2.shortestregrasplength FROM path_cc2"
    result = gdb.execute(sql)
    result = np.asarray(result)
    idpath = [int(x) for x in result[:, 0]]
    startid = [int(x) for x in result[:, 1]]
    goalid = [int(x) for x in result[:, 2]]
    pathnum = [int(x) for x in result[:, 3]]
    shortestregrasplength = [int(x) for x in result[:, 4]]
    drawonce(shortestregrasplength, startid, goalid)
    data4=shortestregrasplength
    #
    #
    # ##############################
    #
    #
    # ax2 = plt.subplot(515)
    # ax2.set_title("on fixture basket time 168")
    # sql = "SELECT pathquick3_fixture.idpathquick3_fixture,pathquick3_fixture.startid,pathquick3_fixture.goalid,pathquick3_fixture.pathnum,pathquick3_fixture.shortestregrasplength FROM pathquick3_fixture"
    # result = gdb.execute(sql)
    # result = np.asarray(result)
    # idpath = [int(x) for x in result[:, 0]]
    # startid = [int(x) for x in result[:, 1]]
    # goalid = [int(x) for x in result[:, 2]]
    # pathnum = [int(x) for x in result[:, 3]]
    # shortestregrasplength = [int(x) for x in result[:, 4]]
    # drawonce(shortestregrasplength, startid, goalid)

    ################################

    # ax2 = plt.subplot(616)
    # ax2.set_title("on fixture basket time 168")
    # sql = "SELECT pathtmp.idpathtmp,pathtmp.startid,pathtmp.goalid,pathtmp.pathnum,pathtmp.shortestregrasplength FROM pathtmp"
    # result = gdb.execute(sql)
    # result = np.asarray(result)
    # idpath = [int(x) for x in result[:, 0]]
    # startid = [int(x) for x in result[:, 1]]
    # goalid = [int(x) for x in result[:, 2]]
    # pathnum = [int(x) for x in result[:, 3]]
    # shortestregrasplength = [int(x) for x in result[:, 4]]
    # drawonce(shortestregrasplength, startid, goalid)

    ##################################

    plt.show()
    ax.set_xticks([])
    ax.set_yticks([])

    ax2 = plt.subplot(211)


    # data = shortestregrasplength
    # bins = np.arange(-1, 10, 1)  # fixed bin size
    # bins = np.linspace(math.ceil(min(data1)),
    #                    math.floor(max(data1)),
    #                    20)
    # width=0.35
    #
    # plt.xlim([min(data1), max(data1)])
    # plt.hist(data1, bins=bins, normed=1,alpha=0.5, label='1:ontable')
    # #plt.hist(data3, bins=bins, normed=1,alpha=0.5, label='3:basket:100')
    # plt.hist(data5, bins=bins, normed=1,alpha=0.5,label='5:basket:168')


    def normfun(x, mu, sigma):
        pdf = np.exp(-((x - mu) ** 2) / (2 * sigma ** 2)) / (sigma * np.sqrt(2 * np.pi))
        return pdf


    def plotpdf(data):
        z = []
        for i in data:
            # if i == -1:
            #     z.append(None)
            # if i == 0:
            #     z.append(None)
            if i != -1 and i != 0:
                z.append(i)

        mean = np.mean(z)
        std = np.std(z)
        y = normfun(x, mean, std)
        return y


    x = np.arange(0, 10, 1)
    y1 = plotpdf(data1)
    y2 = plotpdf(data2)
    y3 = plotpdf(data3)
    y4 = plotpdf(data4)
    # y5 = plotpdf(data5)

    plt.plot(x, y1, c='g', label='1:table')
    plt.plot(x, y2, c='b', label='2:on fixture box drop 18 times')
    plt.plot(x, y3, c='y', label='3:on fixture box drop 50 times')
    plt.plot(x, y4, c='r', label='4:on fixture box drop 100 times')
    # plt.plot(x, y5, c='k', label='5:on fixture box drop 168 times')

    plt.title('regrasp length pdf')
    plt.xlabel('regrasp length')
    plt.ylabel('')
    plt.legend(loc='upper right')
    # plt.show()

    ax2 = plt.subplot(212)


    def plot_successrate(data):
        z = []
        for i in data:
            if i == -1:
                z.append(1)

        return 1 - len(z) / float(len(data))


    x = np.arange(0, 10, 1)
    y1 = plot_successrate(data1)
    y2 = plot_successrate(data2)
    y3 = plot_successrate(data3)
    y4 = plot_successrate(data4)
    # y5 = plot_successrate(data5)

    plt.bar(1, y1, color='g', label='1:table')
    plt.bar(2, y2, color='b', label='2:on fixture box drop 18 times')
    plt.bar(3, y3, color='y', label='3:on fixture box drop 50 times')
    plt.bar(4, y4, color='r', label='4:on fixture box drop 100 times')
    # plt.bar(5, y5, color='k', label='5:on fixture box drop 168 times')

    plt.title('success rate')
    plt.xlabel('experiment num')
    plt.ylabel('success rate')
    plt.legend(loc='upper right')
    plt.show()



    base.run()