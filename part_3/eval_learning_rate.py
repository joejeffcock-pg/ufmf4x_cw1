import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

results = {}
with open('learning_rate_loss.csv','r') as f:
    f.readline()
    for line in f:
        test = [float(v) for v in line.split(',')]
        lr = '{:.3g}'.format(test[0])
        losses = test[1:]
        results[lr] = losses

df = pd.DataFrame(data=results)
mean = df.mean()
std = df.std()
print("Mean:")
print(mean)
print("SD:")
print(std)

df_plot = df.replace(float('inf'), float('nan'))
df_plot = df.dropna(axis='columns')
sns.boxplot(data=df_plot)
plt.show()
