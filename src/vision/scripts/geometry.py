import numpy as np


def centerOfMass(points):
    reshapedPoints = points.reshape(len(points), 2).T
    try:
        cmX = int(reshapedPoints[0].mean())
        cmY = int(reshapedPoints[1].mean())
        cm = np.array([cmX, cmY])
    except ValueError:
        cm = np.array([0, 0])
    return cm


# Covariance
def cov(x, y):
    try:
        xbar, ybar = int(x.mean()), int(y.mean())
    except ValueError:
        xbar, ybar = 0, 0
    return np.sum((x - xbar)*(y - ybar))/(len(x) - 1)


# Covariance matrix
def cov_mat(points):
    reshapedPoints = points.reshape(len(points), 2).T
    xi = reshapedPoints[0]
    yi = reshapedPoints[1]
    return np.array([[cov(xi, xi), cov(xi, yi)],
                     [cov(yi, xi), cov(yi, yi)]])


def floatUnitOrientationVectors(points):
    C = cov_mat(points)
    eigVal, eigVec = np.linalg.eig(C)
    return eigVec


def pixelizeFloatVector(vector):
    vector = vector*100
    return vector.astype(int)


def orientationVectors(points):
    eigVec = floatUnitOrientationVectors(points)
    v1 = pixelizeFloatVector(eigVec[0])
    v2 = pixelizeFloatVector(eigVec[1])
    return v1, v2
