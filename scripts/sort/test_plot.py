import numpy as np 
import matplotlib.pyplot as plt

plt.style.use('plots/test_style.mplstyle')

x = np.linspace(0, 10, 100)
y = np.sin(x)


plt.title(r'Simple plot of $y(x)$')
plt.plot(x, y)
plt.xlabel(r'$x$ axis in (s)')
plt.ylabel(r'$y$ axis in (m)')
plt.grid()
plt.savefig('plots/test')
plt.show()