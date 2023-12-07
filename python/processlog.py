import re
import matplotlib.pyplot as plt


with open('/Users/zhanglei/slam/log/1206/dep.log', 'r') as file:
    lines = file.readlines()

values = []

for log in lines:
    match = re.search(r'dep\((\d+)\):(\d+\.\d+)', log)    
    if match:
        parameter1 = int(match.group(1))
        parameter2 = float(match.group(2))
        values.append(parameter2)

        # print(parameter1, parameter2)        
    
plt.plot(values)
plt.show()