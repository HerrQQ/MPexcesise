import numpy as np
import matplotlib.pyplot as plt

# 定义五次多项式的系数
a = 1
b = -2
c = 3
d = -4
e = 5
f = 6

# 定义x的取值范围
x = np.linspace(-10, 10, 400)

# 计算对应的y值
y = a * x**5 + b * x**4 + c * x**3 + d * x**2 + e * x + f

# 绘制图形
plt.plot(x, y)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.title('Five Degree Polynomial without Inflection Points')
plt.grid(True)
plt.show()
