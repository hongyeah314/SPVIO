import pandas as pd

# 假设你的数据存在一个名为'data.csv'的CSV文件中
df = pd.read_csv('/Users/zhanglei/SLAM/log/1212/ceres2.txt', sep="                          ", header=None)

# 现在，你可以通过列名来获取这个数字
number = df['Initial'][0]
print(number)