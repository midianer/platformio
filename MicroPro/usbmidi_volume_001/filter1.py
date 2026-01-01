import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots()

dt = 0.01
t = np.arange(dt, 0.50, dt)

ax.semilogx(t, np.exp(-t / 0.5))
#ax.semilogy(4*dt, 10**dt, 'o--')
ax.grid()

plt.show()
