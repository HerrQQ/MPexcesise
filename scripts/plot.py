import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches

# final trace 
data_x = [
0.009998,
1.01573,
2.0223,
3.02987,
4.03861,
5.04862,
6.05996,
7.07268,
8.08677,
9.10222,
10.119,
11.137,
12.1561,
13.1762,
14.1972,
15.219,
16.2414,
17.2643,
18.2874,
19.3106,
20.3338,
21.3567,
22.3792,
23.4011,
24.4223,
25.4426,
26.4618,
27.4799,
28.4966,
29.5118,
29.5105,
30.5227,
31.5332,
32.542,
33.5489,
34.5539,
35.557,
36.5582,
37.5574,
38.5546,
39.55,
40.5434,
41.5351,
42.5249,
43.5131,
43.4906,
44.4761,
45.4603,
46.4432,
47.425,
48.4057,
49.3856,
50.3647,
51.3432,
51.3167,
52.295,
53.2731,
54.2513,
55.2297,
56.2084,
57.1875,
58.1674,
58.1461,
59.1279,
60.1106,
61.0944,
62.0794,
63.0657,
63.0527,
64.0417,
65.0322,
66.024,
67.0172,
68.0116,
68.0073,
69.0042,
70.0021,
71.0008,
72.0002,
72,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73,
73]

x_ans = np.array(data_x)


data_y = [9.997e-05,
0.000566855,
0.00277648,
0.00790806,
0.0170798,
0.0313493,
0.0517139,
0.0791117,
0.114422,
0.158465,
0.212006,
0.275753,
0.350358,
0.43642,
0.534484,
0.645046,
0.76855,
0.90539,
1.05591,
1.22043,
1.39918,
1.59239,
1.80024,
2.02285,
2.26033,
2.51273,
2.78008,
3.06238,
3.35958,
3.67163,
3.67604,
4.00721,
4.35289,
4.71291,
5.08711,
5.47527,
5.87718,
6.29263,
6.72136,
7.16313,
7.61767,
8.08472,
8.56401,
9.05526,
9.55819,
9.60765,
10.123,
10.6492,
11.186,
11.7331,
12.2902,
12.8571,
13.4336,
14.0194,
14.0688,
14.6625,
15.2648,
15.8756,
16.4948,
17.1221,
17.7575,
18.4008,
18.4356,
19.0843,
19.7407,
20.4047,
21.0765,
21.7558,
21.7755,
22.4601,
23.1525,
23.8529,
24.5614,
25.2781,
25.2842,
26.0077,
26.7402,
27.4818,
28.233,
28.2332,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994,
28.994]

y_ans = np.array(data_y)









# 创建数据
#1
d0= -0.5
d1= -0.0201
d2= -0.00477639
d3= 0.00773737
d4= -0.00107383
d5= 4.1796e-05
x2 = np.arange(1,11,1)
#2
ad0=-0.5
ad1=-0.0201
ad2=-0.00477639
ad3=0.00165963
ad4=-0.00010447
ad5=1.95332e-06
ax2 = np.arange(1,21,1)
#3
bd0=-0.5
bd1=-0.0201
bd2=-0.00477639
bd3=0.000803611
bd4=-3.15897e-05
bd5=3.82384e-07
bx2 = np.arange(1,31,1)
#4
cd0=-0.5
cd1=-0.0201
cd2=-0.00477639
cd3=0.000515604
cd4=-1.45922e-05
cd5=1.29917e-07 
cx2 = np.arange(1,41,1)
#5
dd0=-0.5
dd1=-0.0201
dd2=-0.00477639
dd3=0.000377445
dd4=-8.32318e-06
dd5=5.8513e-08
dx2 = np.arange(1,51,1)
#6
ed0=-0.5
ed1=-0.0201
ed2=-0.00477639
ed3=0.000297422
ed4=-5.3688e-06
ed5=3.11695e-08
ex2 = np.arange(1,61,1)
#7
fd0=-0.5
fd1=-0.0201
fd2=-0.00477639
fd3=0.000245442
fd4=-3.7499e-06
fd5=1.85397e-08
fx2 = np.arange(1,71,1)
#8
gd0=-0.5
gd1=-0.0201
gd2=-0.00477639
gd3=0.000209005
gd4=-2.76822e-06
gd5=1.19178e-08
gx2 = np.arange(1,81,1)
y=d0+d1*x2+d2*pow(x2,2)+d3*pow(x2,3)+d4*pow(x2,4)+d5*pow(x2,5)
ay=ad0+ad1*ax2+ad2*pow(ax2,2)+ad3*pow(ax2,3)+ad4*pow(ax2,4)+ad5*pow(ax2,5)
by=bd0+bd1*bx2+bd2*pow(bx2,2)+bd3*pow(bx2,3)+bd4*pow(bx2,4)+bd5*pow(bx2,5)
cy=cd0+cd1*cx2+cd2*pow(cx2,2)+cd3*pow(cx2,3)+cd4*pow(cx2,4)+cd5*pow(cx2,5)
dy=dd0+dd1*dx2+dd2*pow(dx2,2)+dd3*pow(dx2,3)+dd4*pow(dx2,4)+dd5*pow(dx2,5)
ey=ed0+ed1*ex2+ed2*pow(ex2,2)+ed3*pow(ex2,3)+ed4*pow(ex2,4)+ed5*pow(ex2,5)
fy=fd0+fd1*fx2+fd2*pow(fx2,2)+fd3*pow(fx2,3)+fd4*pow(fx2,4)+fd5*pow(fx2,5)
gy=gd0+gd1*gx2+gd2*pow(gx2,2)+gd3*pow(gx2,3)+gd4*pow(gx2,4)+gd5*pow(gx2,5)

fig, ax = plt.subplots()


rectangle = patches.Rectangle((-1, -0.125), 2, 0.25, linewidth=1, edgecolor='r', facecolor='none')
ax.add_patch(rectangle)

zero_array = np.zeros(80)


plt.plot(x2, y, '--g', label='MP0')  
plt.plot(ax2, ay, '--g', label='MP1')
plt.plot(bx2, by, '--g', label='MP2')
plt.plot(cx2, cy, '--g', label='MP3')
plt.plot(dx2, dy, '--g', label='MP4')
plt.plot(ex2, ey, '--g', label='MP5')
plt.plot(fx2, fy, '--g', label='MP6')
plt.plot(gx2, gy, '--g', label='MP7')
plt.plot(gx2, zero_array, '-r', label='Reference_FRE')
plt.plot(x_ans, y_ans, '--b', label='MP_VCS') 


c0=0.5
c1=0.02
c2=5e-3
c3=1e-6
x = np.arange(1,81,1)
y1=c0+c1*x+c2*pow(x,2)+c3*pow(x,3)
plt.plot(x, y1, '-r', label='Reference')  





plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()

