import math
import numpy as np
import exceptions as ep
import utils.robotmath as rm

def eubik(pos, armid="rgt"):
    """
    compute the euristic waist rotation
    ew = euristic waist

    :param pos: object position
    :return: waistangle in degree

    author: weiwei
    date: 20161111
    """

    anglecomponent1 = 0
    try:
        anglecomponent1 = math.asin(145/np.linalg.norm(pos[0:2]))
    except:
        pass
    waistangle = (math.atan2(pos[1], pos[0]) + anglecomponent1)*180/math.pi
    if armid=="lft":
        waistangle = -waistangle
    return waistangle


def jacobian(nxtrobot, armid="rgt"):
    """
    compute the jacobian matrix of rgt or lft arm

    :param nxtrobot: see the nextage.NxtRobot class
    :param armid: a string indicating "rgt" or "lft"
    :return: armjac a 6-by-6 ndarray

    author: weiwei
    date: 20161111
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    armlj = nxtrobot.rgtarm
    if armid == "lft":
        armlj = nxtrobot.lftarm

    armjac = np.zeros((6,6))
    for i in range(6):
        a = np.dot(armlj[i+1]["rotmat"], armlj[i+1]["rotax"])
        armjac[:, i] = np.append(np.cross(a, armlj[6]["linkpos"]-armlj[i+1]["linkpos"]), a)

    return armjac


def manipulability(nxtrobot, armid="rgt"):
    """
    compute the yasukawa manipulability of rgt or lft arm (6-dof)

    :param nxtrobot: see the nextage.NxtRobot class
    :param armid: a string indicating "rgt" or "lft"
    :return:
    """

    armjac = jacobian(nxtrobot, armid)
    return math.sqrt(np.linalg.det(np.dot(armjac, armjac.transpose())))


def tcperror(nxtrobot, tgtpos, tgtrot, armid="rgt"):
    """
    compute the error of a specified (rgt or lft) tool point center to its goal

    :param nxtrobot: see the nextage.NxtRobot class
    :param armid: a string indicating "rgt" or "lft"
    :param tgtpos: the position of the goal
    :param tgtrot: the rotation of the goal
    :return: a 1-by-6 vector where the first three indicates the displacement in pos,
                the second three indictes the displacement in rot

    author: weiwei
    date: 20161111
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    armlj = nxtrobot.rgtarm
    if armid == "lft":
        armlj = nxtrobot.lftarm

    deltapos = tgtpos - armlj[6]["linkend"]
    deltarot = np.dot(tgtrot, armlj[6]["rotmat"].transpose())

    anglesum = np.trace(deltarot)
    if anglesum is 3:
        deltaw = np.array([0,0,0])
    else:
        # revised on 20161216 at sapporo
        # compute the geodesic distance of two rotationmatrix
        # logarithm of R'R
        # see pygeometry.geodesic_distance_for_rotations for details
        nominator = anglesum-1
        if nominator > 2:
            nominator = 2
        if nominator < -2:
            nominator = -2
        theta = math.acos(nominator/2.0)
        if theta == 0:
            deltaw = np.array([0,0,0])
        else:
            deltaw = (theta/(2*math.sin(theta)))*(np.array([deltarot[2,1]-deltarot[1,2], \
                                                            deltarot[0,2]-deltarot[2,0], \
                                                            deltarot[1,0]-deltarot[0,1]]))

    return np.append(deltapos, deltaw)


def numik(nxtrobot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik numerically for the specified armid

    :param nxtrobot: see nextage.NxtRobot class
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot: the orientation of the goal, 3-by-3 numpyndarray
    :param armid: a string "rgt" or "lft" indicating the arm that will be solved
    :return: armjnts: a 1-by-6 numpy ndarray

    author: weiwei
    date: 20161111
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    # armlj = nxtrobot.rgtarm
    # if armid == "lft":
    #     armlj = nxtrobot.lftarm

    # stablizer
    steplength = 30
    armjntssave = nxtrobot.getarmjnts(armid)
    armjntsiter = armjntssave.copy()
    for i in range(500):
        armjac = jacobian(nxtrobot, armid)
        if abs(np.linalg.det(armjac))>1e-6:
            err = tcperror(nxtrobot, tgtpos, tgtrot, armid)
            dq = steplength*(np.linalg.solve(armjac, err))
        else:
            print "The Jacobian Matrix of the specified arm is at singularity"
            nxtrobot.movearmfk6(armjntssave, armid)
            return None
        if np.linalg.norm(err)<1e-4:
            armjntsreturn = nxtrobot.getarmjnts(armid)
            nxtrobot.movearmfk(armjntssave, armid)
            return armjntsreturn
        else:
            armjntsiter += dq
            armjntsiter = rm.cvtRngPM180(armjntsiter)
            if nxtrobot.chkrng(armjntsiter) or i < 10:
                nxtrobot.movearmfk(armjntsiter, armid)
                import nxtplot
                # nxtplot.plotstick(base.render, nxtrobot)
                # nxtmnp = nxtplot.genNxtmnp_nm(nxtrobot,plotcolor=[.5,.5,0.1,.2])
                # nxtmnp.reparentTo(base.render)
                # nxtmnp = nxtplot.genNxtmnp(nxtrobot)
                # nxtmnp.reparentTo(base.render)
            else:
                nxtrobot.movearmfk(armjntssave, armid)
                # print "No feasible IK"
                return None
    nxtrobot.movearmfk(armjntssave, armid)

def numikr(nxtrobot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik of the specified arm, waist is included (r means redundant)

    :param nxtrobot:
    :param tgtpos:
    :param tgtrot:
    :param armid:
    :return: a 1,1-by-6 numpy ndarray where the first element is the waist rot angle

    author: weiwei
    date: 20161216, sapporo
    """

    # anglewi means the init angle of waist
    anglewi = nxtrobot.getjntwaist()
    armjntb = eubik(tgtpos, armid)
    nxtrobot.movewaist(armjntb)
    armjnts6 = numik(nxtrobot, tgtpos, tgtrot, armid)
    if armjnts6 is None:
        nxtrobot.movewaist(anglewi)
        return None
    else:
        nxtrobot.movewaist(anglewi)
        armjnts7 = [armjntb, armjnts6]
        return armjnts7

if __name__=="__main__":
    pos = [300,300,0]
    print eubik(pos)

    try:
        print math.asin(145/np.linalg.norm(pos[0:1]))
    except:
        print "nontriangle"