import numpy
#array([ 18.03479719, -37.00420825,   3.45887193,   5.93619392,
#         2.51073548, -29.67857742, -27.9399575 ,  26.78882276,
#       -25.62662082,  29.64418211])
# parameter.
num_samples = 10
min_distance = 5.0
X_min, X_max = -39.0, 39.0
Y_min, Y_max = -44.0, 44.0
# END parameter.

counter = 1
X = [0]
Y = [0]
while counter < num_samples:
    print "adding ", counter
    x = numpy.random.uniform(X_min, X_max)
    y = numpy.random.uniform(Y_min, Y_max)
    to_be_added = True
    for i in xrange(len(X)):
        a = numpy.array((x, y))
        b = numpy.array((X[i], Y[i]))
        dist = numpy.linalg.norm(a-b)
        if dist < min_distance:
            to_be_added = False
            break
    if to_be_added:
        X.append(x) 
        Y.append(y) 
        counter += 1
print X
print Y
import matplotlib.pyplot as plt
plt.plot(X, Y, 'ro')
plt.show()
