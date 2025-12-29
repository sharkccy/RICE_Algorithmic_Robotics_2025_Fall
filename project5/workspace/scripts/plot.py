import matplotlib.pyplot as plt
import numpy as np

# x_labels = ['N=1', 'N=4', 'N=8', 'N=32', 'N=64']
x_labels = ['T=1', 'T=4', 'T=8', 'T=32', 'T=64']
x = range(len(x_labels))
# y_len = [2.814, 2.120, 2.024, 1.802, 1.757]

y = [180.549, 149.458, 113.604, 99.901, 99.448]

# plt.figure(figsize=(14, 5))
# plt.subplot(1, 2, 1)
# plt.plot(x, y_len, marker='o', color='blue')
# plt.title('Average Path Length Given N', pad=10)
# plt.xlabel('Number of candidate initialized paths', labelpad=10)
# plt.ylabel('Average Path Length (m)', labelpad=20)
# plt.xticks(x, x_labels)
# plt.yticks(np.arange(0, 3.5, 0.5))
# plt.grid(True)

# plt.subplot(1, 2, 2)
plt.plot(x, y, marker='o', color='red')
plt.title('Average Time Given Threads Number', pad=10)
plt.xlabel('Number of Threads', labelpad=10)
plt.ylabel('Average Time (s)', labelpad=20)
plt.xticks(x, x_labels)
plt.yticks(np.arange(0, 250, 25))
plt.grid(True)

plt.savefig('Average Time Given Threads Number.png')
plt.tight_layout(pad=3.0)
plt.show()