import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

results = {}
with open('tolerance_times.csv','r') as f:
    f.readline()
    for line in f:
        test = [float(v) for v in line.split(',')]
        label = '{:.3g},{:.3g}'.format(test[0], test[1])
        times = test[2:]
        results[label] = times

print(results.keys())
df = pd.DataFrame(data=results)
print(df)

success = df.ge(0)
print(success)
print(success.sum()/1000 * 100)

df = df.replace(-1, float('nan'))
mean = df.mean()
std = df.std()
print("Mean:")
print(mean)
print("SD:")
print(std)

sns.boxplot(data=df)
plt.show()
