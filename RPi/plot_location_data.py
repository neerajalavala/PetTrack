from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

#location_data = pd.read_csv("Location_Mila_[TEST]_[room#X].csv")
bedroom_df = pd.read_csv("Location_Mila_[TEST]_[room:bedroom].csv")
bath1_df = pd.read_csv("Location_Mila_[TEST]_[room:bath1].csv")
laundry_df = pd.read_csv("Location_Mila_[TEST]_[room:laundry].csv")
bath2_df = pd.read_csv("Location_Mila_[TEST]_[room:bath2].csv")
office_df = pd.read_csv("Location_Mila_[TEST]_[room:office].csv")
living_df = pd.read_csv("Location_Mila_[TEST]_[room:living].csv")
dining_df = pd.read_csv("Location_Mila_[TEST]_[room:dining].csv")

fig, ax = plt.subplots(1,1)
#ax.scatter(x=location_data. iloc[:, 0],y=location_data. iloc[:, 1])
ax.scatter(x=bedroom_df.iloc[:,0], y=bedroom_df.iloc[:,1], c="red", label="bedroom")
ax.scatter(x=bath1_df.iloc[:,0], y=bath1_df.iloc[:,1], c="orange", label="bath1")
ax.scatter(x=laundry_df.iloc[:,0], y=laundry_df.iloc[:,1], c="green", label="laundry")
ax.scatter(x=bath2_df.iloc[:,0], y=bath2_df.iloc[:,1], c="blue", label="bath2")
ax.scatter(x=office_df.iloc[:,0], y=office_df.iloc[:,1], c="purple", label="office")
ax.scatter(x=living_df.iloc[:,0], y=living_df.iloc[:,1], c="brown", label="living")
ax.scatter(x=dining_df.iloc[:,0], y=dining_df.iloc[:,1], c="black", label="dining")

ax.axis('equal')
ax.set_aspect('equal','box')
plt.legend()
plt.show()

#means = location_data.mean(axis=0)
#TODO: do cdf plot with means subtracted from the data