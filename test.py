import pandas as pd

df = pd.read_csv('Gripper_position.csv')
df = df.drop(columns=['Unnamed: 0'])
print(df.head())
df.to_csv("Gripper_position.csv",index=None)