import re
import matplotlib.pyplot as plt
import pandas as pd


data = pd.read_csv('/Users/zhanglei/code/SPVIO/python/Cpp/build/data.txt', sep=" ", header=None)

# for i in range(data.shape[1]):
#     plt.plot(data[i])

plt.plot(data[0],data[1])


plt.show()