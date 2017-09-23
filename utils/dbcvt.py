# for saving data into mysql
from panda3d.core import *

def mat3ToStr(pdmat3):
    """
    convert a mat3 matrix to a string like e00, e01, e02, ...

    :param pdmat4:
    :return: a string

    author: jiayao
    date: 20160809, tsukuba
    """

    row0 = pdmat3.getRow(0)
    row1 = pdmat3.getRow(1)
    row2 = pdmat3.getRow(2)


    return ','.join(str(e) for e in row0)+','+','.join(str(e) for e in row1)+\
           ','+','.join(str(e) for e in row2)



def LVecBase3fToStr(LVecBase3f):
    """
    convert a LVecBase3f to a string like e00, e01, e02, ...

    :param :LVecBase3f
    :return: a string

    author: jiayao
    date: 20160810, tsukuba
    """

    x = LVecBase3f.getX()
    y = LVecBase3f.getY()
    z = LVecBase3f.getZ()

    return str(x)+','+str(y)+','+str(z)

def mat4ToStr(pdmat4):
    """
    convert a mat4 matrix to a string like e00, e01, e02, ...

    :param pdmat4:
    :return: a string

    author: weiwei
    date: 20161212, tsukuba
    """

    row0 = pdmat4.getRow(0)
    row1 = pdmat4.getRow(1)
    row2 = pdmat4.getRow(2)
    row3 = pdmat4.getRow(3)

    return ','.join(str(e) for e in row0)+','+','.join(str(e) for e in row1)+\
           ','+','.join(str(e) for e in row2)+','+','.join(str(e) for e in row3)

def strToMat4(dbstr):
    """
    convert a string like e00, e01, e02, ... into Mat4

    :param str:
    :return: panda Mat4

    author: weiwei
    date: 20161212, tsukuba
    """

    exx = dbstr.split(',')
    exxdecimal = map(float, exx)
    assert(len(exxdecimal) is 16)
    return Mat4(exxdecimal[0], exxdecimal[1], exxdecimal[2], exxdecimal[3],
                exxdecimal[4], exxdecimal[5], exxdecimal[6], exxdecimal[7],
                exxdecimal[8], exxdecimal[9], exxdecimal[10], exxdecimal[11],
                exxdecimal[12], exxdecimal[13], exxdecimal[14], exxdecimal[15])

def strToMat3(dbstr):
    """
    convert a string like e00, e01, e02, ... into Mat3

    :param str:
    :return: panda Mat4

    author: jiayao
    date: 20170810, tsukuba
    """

    exx = dbstr.split(',')
    exxdecimal = map(float, exx)
    assert(len(exxdecimal) is 9)

    return Mat3(exxdecimal[0], exxdecimal[1], exxdecimal[2],
               exxdecimal[3], exxdecimal[4], exxdecimal[5],
              exxdecimal[6], exxdecimal[7], exxdecimal[8])





def v3ToStr(v3):
    """
    convert a vbase3 vector to a string like v0,v1,v2

    :param v3:
    :return:
    """

    return ','.join(str(e) for e in v3)

# def strToVector3(dbstr):
#     """
#     convert a string like v0,v1,v2 to a v3
#
#     :param string
#     :return: vector3,no vbase structure.
#     author: jiayao
#     date: 20170811, tsukuba
#     """
#
#     exx = dbstr.split(',')
#     exxdecimal = map(float, exx)
#     assert(len(exxdecimal) is 3)
#     return (exxdecimal[0], exxdecimal[1], exxdecimal[2])

def strToV3(dbstr):
    """

    :param dbstr: v3
    :return:
    """

    exx = dbstr.split(',')
    exxdecimal = map(float, exx)
    assert(len(exxdecimal) is 3)
    return VBase3(exxdecimal[0], exxdecimal[1], exxdecimal[2])

