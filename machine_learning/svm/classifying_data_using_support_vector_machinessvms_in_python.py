#-*- coding:utf-8 -*-
# reference to https://www.geeksforgeeks.org/classifying-data-using-support-vector-machinessvms-in-python/

# reference to https://stackoverflow.com/questions/65898399/no-module-named-sklearn-datasets-samples-generator
# python -m pip install -U scikit-learn
from sklearn.datasets import make_blobs

# creating datasets X containing n_samples
# Y containing two classes
X, Y = make_blobs(n_samples=500, centers = 2, random_state=0, cluster_std=0.40)

import matplotlib.pyplot as plt
#plotting scatters

plt.scatter(X[:, 0], X[:, 1], c=Y, s = 50, cmap = 'spring')
plt.show()

#creating line space between -1 to 3.5
import numpy as np
xfit = np.linespace(-1, 3.5)

#plot scatter
plt.scatter(X[:, 0], X[:, 1], c=Y, s=50, cmap='spring')
# plot a line between the different sets of data
for m, b, d in [(1, 0.65, 0.33), (0.5, 1.6, 0.55), (-0.2, 2.9, 0.2)]:
    yfit = m * xfit + b
    plt.plot(xfit, yfit, '-k')
    plt.fill_between(xfit, yfit - d, yfit + d, edgecolor='none', \
         color='#AAAAAA', alpha=0.4)
  
plt.xlim(-1, 3.5)
plt.show()

# importing required libraries
import pandas as pd

# reading csv file and extracting class column to y.
x = pd.read_csv("./cancer.csv")
a = np.array(x)
y = a[:,30] # classes having 0 and 1

# extracting two features
x = np.column_stack((x.malignant,x.benign))

# 569 samples and 2 features
x.shape

print (x),(y)

# import support vector classifier
# "Support Vector Classifier"
from sklearn.svm import SVC
clf = SVC(kernel='linear')

# fitting x samples and y classes
clf.fit(x, y)