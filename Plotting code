import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn import linear_model
import statsmodels.api as sm
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
#change the directory infromation to load the csv file.(add the path in the double quotes)	
gt = pd.read_csv(r"C:\Users\pnidh\Desktop\log.csv")
gt.head()
gt.shape
est_traj_fig = plt.figure()
ax = est_traj_fig.add_subplot(111)
ax.plot(gt.x_e,gt.y_e, label='Estimated')
ax.plot(gt.x_gt,gt.y_gt, label='Ground Truth')
