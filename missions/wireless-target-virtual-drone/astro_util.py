from __future__ import generators
import numpy as np
import math
import utm
from math import sqrt



# Determines whether two circles collide and, if applicable,
# the points at which their borders intersect.
# Based on an algorithm described by Paul Bourke:
# http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
# Arguments:
#   P0 (complex): the centre point of the first circle
#   P1 (complex): the centre point of the second circle
#   r0 (numeric): radius of the first circle
#   r1 (numeric): radius of the second circle
# Returns:
#   False if the circles do not collide
#   True if one circle wholly contains another such that the borders
#       do not overlap, or overlap exactly (e.g. two identical circles)
#   An array of two complex numbers containing the intersection points
#       if the circle's borders intersect.
def IntersectPoints(P0, P1, r0, r1):
    if type(P0) != complex or type(P1) != complex:
        raise TypeError("P0 and P1 must be complex types")
    # d = distance
    d = sqrt((P1.real - P0.real)**2 + (P1.imag - P0.imag)**2)
    # n**2 in Python means "n to the power of 2"
    # note: d = a + b

    if d > (r0 + r1):
        return False
    elif d < abs(r0 - r1):
        return True
    elif d == 0:
        return True
    else:
        a = (r0**2 - r1**2 + d**2) / (2 * d)
        b = d - a
        h = sqrt(r0**2 - a**2)
        P2 = P0 + a * (P1 - P0) / d

        i1x = P2.real + h * (P1.imag - P0.imag) / d
        i1y = P2.imag - h * (P1.real - P0.real) / d
        i2x = P2.real - h * (P1.imag - P0.imag) / d
        i2y = P2.imag + h * (P1.real - P0.real) / d

        i1 = complex(i1x, i1y)
        i2 = complex(i2x, i2y)

        return [i1, i2]

# closest pairs by divide and conquer
# David Eppstein, UC Irvine, 7 Mar 2002







def closestpair(L):
    def square(x): return x*x
    def sqdist(p,q): return square(p[0]-q[0])+square(p[1]-q[1])

    # Work around ridiculous Python inability to change variables in outer scopes
    # by storing a list "best", where best[0] = smallest sqdist found so far and
    # best[1] = pair of points giving that value of sqdist.  Then best itself is never
    # changed, but its elements best[0] and best[1] can be.
    #
    # We use the pair L[0],L[1] as our initial guess at a small distance.
    best = [sqdist(L[0],L[1]), (L[0],L[1])]

    # check whether pair (p,q) forms a closer pair than one seen already
    def testpair(p,q):
        d = sqdist(p,q)
        if d < best[0]:
            best[0] = d
            best[1] = p,q

    # merge two sorted lists by y-coordinate
    def merge(A,B):
        i = 0
        j = 0
        while i < len(A) or j < len(B):
            if j >= len(B) or (i < len(A) and A[i][1] <= B[j][1]):
                yield A[i]
                i += 1
            else:
                yield B[j]
                j += 1

    # Find closest pair recursively; returns all points sorted by y coordinate
    def recur(L):
        if len(L) < 2:
            return L
        split = np.int(len(L)/2)
        splitx = L[split][0]
        L = list(merge(recur(L[:split]), recur(L[split:])))

        # Find possible closest pair across split line
        # Note: this is not quite the same as the algorithm described in class, because
        # we use the global minimum distance found so far (best[0]), instead of
        # the best distance found within the recursive calls made by this call to recur().
        # This change reduces the size of E, speeding up the algorithm a little.
        #
        E = [p for p in L if abs(p[0]-splitx) < best[0]]
        for i in range(len(E)):
            for j in range(1,8):
                if i+j < len(E):
                    testpair(E[i],E[i+j])
        return L

    L.sort()
    recur(L)
    return best[1]
