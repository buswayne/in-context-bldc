import numpy as np
import matplotlib.pyplot as plt


ind = np.arange(100)-50
lim = 25
for i in ind:
    j =np.clip(i, -lim, lim)
    print(j)

# plt.figure()
# x = np.arange(len(theta_show))
# plt.plot(x,theta_show)
# plt.plot(x,theta_show_wrap)
# plt.show()
