import math

r0 = 2
h0 = 1
k0 = 3

x1 = h0
y1 = k0 + r0
x2 = h0
y2 = k0 - r0
x3 = h0 + r0
y3 = k0

ad12 = -2 * (y2 - y1)
a1 = (x1*x1 - x2*x2) / ad12
a2 = (y1*y1 - y2*y2) / ad12
a3 = (2 * (x2 - x1)) / ad12

ad13 = -2 * (y3 - y1)
a4 = (x1*x1 - x3*x3) / ad13
a5 = (y1*y1 - y3*y3) / ad13
a6 = (2 * (x3 - x1)) / ad13
h = (a1 - a4 + a2 - a5) / (a6 - a3)
print(h)

b1 = a1
b2 = a2
b3 = a3
k = b1 + b2 + b3 * h
print(k)

c1 = pow(x1-h, 1)
c2 = pow(y1-k, 2)
r = math.sqrt(c1 + c2)
print(r)
