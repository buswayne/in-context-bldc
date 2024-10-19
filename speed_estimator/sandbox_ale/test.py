import numpy as np
import matplotlib.pyplot as plt

theta = -5
theta_show = [0]
theta_show_wrap = [0]

for i in np.arange(1000):
    theta = theta + 0.1
    theta_show.append(theta)
    theta_wrapped = np.remainder(theta, 2*np.pi)
    theta_show_wrap.append(theta_wrapped)

plt.figure()
x = np.arange(len(theta_show))
plt.plot(x,theta_show)
plt.plot(x,theta_show_wrap)
plt.show()
