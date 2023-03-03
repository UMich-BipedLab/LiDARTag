import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

fig = plt.figure(figsize=(6.4, 4.8), dpi=100,
                 facecolor='w', linewidth=0, edgecolor='w')
ax = fig.add_subplot(111, title='pose rotation result', ylim=(-5, 5))
df = pd.read_csv('../outputs/results_pandar40p_10m_angle_change.csv')
df = df.assign(yaw_positive=0)
df = df.assign(yaw_negative=0)

for i in df['yaw']:
    if (i > 0) :
        df.append({'yaw_positive': i},  ignore_index=True)
    else :
        df.append({'yaw_negative': i},  ignore_index=True)

p = plt.yticks( np.arange(-5, 5, 0.5))
p = plt.plot(df['valid'], label="valid")
p = plt.plot(df['id'], label="id")
p = plt.plot(df['x'], label="x")
p = plt.plot(df['y'], label="y")
p = plt.plot(df['z'], label="z")
p = plt.plot(df['roll'], label="pose_roll")
p = plt.plot(df['pitch'], label="pose_pitch")
p = plt.plot(df['yaw_positive'], label="pose_yaw_positive")
p = plt.plot(df['yaw_negative'], label="pose_yaw_negative")
p = plt.plot(df['tag_size'], label="tag_size")

# p = plt.plot([0, len(df)],[2.94, 2.94], "red", linewidth=1.0, linestyle='dashed',)
# p = plt.plot([0, len(df)],[2.47, 2.47], "red", linewidth=1.0, linestyle='dashed')
# p = plt.plot([0, len(df)],[0.93, 0.93], "red", linestyle='dashed', label="parallel line")
# p = plt.plot([0, len(df)],[-0.93, -0.93], "red", linestyle='dashed')

p = plt.legend(loc="best")
plt.show()
