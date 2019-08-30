#
#

import numpy as np
import trjgen.guidance_helper as gh

p = np.array([1, -1, 0]);
v = np.array([1, -1, 0]);
a = np.array([0, 0, 0]);

print("Integrating 2s forward with constant acceleration:")
a[0] = 4
a[1] = -4

print(p)
print(v)
print(a)
out = gh.Integration(p, v, a, 2.0, 0.1, 1)
print(out)
#print(tfinal)


print("Integrating 1s backward with constant decelleration:")
p = out[0:3]
v = out[3:6]

print(p)
print(v)
print(a)
out = gh.Integration(p, v, a, 2.0, 0.1, -1)
print(out)




