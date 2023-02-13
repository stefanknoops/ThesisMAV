import os
import bagpy
from bagpy import bagreader
import pandas as pd

cwd = os.getcwd()
b = bagreader(os.path.join(cwd, 'Experiments/dronetest_7/_2022-10-13-10-10-08.bag'))
print(b.topic_table)

FOE_MSG = b.message_by_topic('/dvs/imu')
df_foe = pd.read_csv(FOE_MSG)

print(df_foe.head())