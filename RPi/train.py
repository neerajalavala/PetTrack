import pandas as pd

df = pd.read_csv('IMU_data_new.csv', header=None)
df.loc[df[1]<0, 1] = (df.loc[df[1]<0]+180)/360
df.loc[df[1]>0, 1] = (df.loc[df[1]>0])/360
df.loc[df[2]<0, 2] = (df.loc[df[2]<0]+180)/360
df.loc[df[2]>0, 2] = (df.loc[df[2]>0])/360
df.loc[df[3]<0, 3] = (df.loc[df[3]<0]+180)/360
df.loc[df[3]>0, 3] = (df.loc[df[3]>0])/360

df[4] = df[4]/9.8
df[5] = df[5]/9.8
df[6] = df[6]/9.8
dataTrain = df[[1,2,3,4,5,6]].to_numpy()
dataTarget = df[7].to_numpy()

from sklearn import model_selection
X_train,X_valid,y_train,y_valid = model_selection.train_test_split(dataTrain,dataTarget, test_size = 0.3, random_state = 10)
from sklearn.neighbors import KNeighborsClassifier
knn = KNeighborsClassifier(n_neighbors=4)
knn.fit(X_train, y_train)
ypred = knn.predict(X_valid)

'''
=======
dataTrain = df[[1,2,4,6]].to_numpy()
dataTarget = df[7].to_numpy()
>>>>>>> 96664c9d9d25ec16e589ca73fc571c62336c8793
from sklearn import model_selection
X_train,X_valid,y_train,y_valid = model_selection.train_test_split(dataTrain,dataTarget, test_size = 0.33, random_state = 42)
from sklearn.ensemble import RandomForestClassifier
from sklearn import metrics
randomForest = RandomForestClassifier(n_estimators = 200,
                            min_samples_leaf = 10,
                            n_jobs = -1, max_depth=3)
randomForest.fit(X_train,y_train)
randomPredict = randomForest.predict(X_valid)
print(' {:.2f} out of 1 on test data'.format(metrics.accuracy_score(randomPredict, y_valid)))
<<<<<<< HEAD
'''

import pickle
filename = 'new_model.sav'
pickle.dump(knn, open(filename, 'wb'))
'''
=======
import pickle
filename = 'new_model.sav'
pickle.dump(randomForest, open(filename, 'wb'))
>>>>>>> 96664c9d9d25ec16e589ca73fc571c62336c8793
'''