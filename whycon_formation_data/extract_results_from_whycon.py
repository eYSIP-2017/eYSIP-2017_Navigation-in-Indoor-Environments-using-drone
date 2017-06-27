from __future__ import print_function
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier
from sklearn.svm import SVC
from sklearn import metrics
import pickle

marker_dfs = dict()
line_dfs = dict()
angles_dfs = dict()
slope_dfs = dict()
# for i in range(5):
#     df = pd.read_csv('{}_{}_{}.txt'.format(i, i+1, i+2))
#     strings = ['{}_{}'.format(i, 0+i), '{}_{}'.format(i, 1+i), '{}_{}'.format(i, 2+i)]
#     marker_dfs[strings[0]] = df[df['index'] == 0].reset_index()
#     marker_dfs[strings[1]] = df[df['index'] == 1].reset_index()
#     marker_dfs[strings[2]] = df[df['index'] == 2].reset_index()
    
#     lines = ['{}_{}'.format(0+i, 1+i), '{}_{}'.format(0+i, 2+i), '{}_{}'.format(1+i, 2+i)]
#     # line_dfs['{}_{}_{}'.format(i, 0+i, 1+i)] = np.sqrt(np.square(marker_dfs[strings[0]].y - marker_dfs[strings[1]].y) + np.square(marker_dfs[strings[0]].z - marker_dfs[strings[1]].z))
#     # line_dfs['{}_{}_{}'.format(i, 0+i, 2+i)] = np.sqrt(np.square(marker_dfs[strings[0]].y - marker_dfs[strings[2]].y) + np.square(marker_dfs[strings[0]].z - marker_dfs[strings[2]].z))
#     # line_dfs['{}_{}_{}'.format(i, 1+i, 2+i)] = np.sqrt(np.square(marker_dfs[strings[1]].y - marker_dfs[strings[2]].y) + np.square(marker_dfs[strings[1]].z - marker_dfs[strings[2]].z))
#     line_dfs[lines[0]] = np.sqrt(np.square(marker_dfs[strings[0]].y - marker_dfs[strings[1]].y) + np.square(marker_dfs[strings[0]].z - marker_dfs[strings[1]].z))
#     line_dfs[lines[1]] = np.sqrt(np.square(marker_dfs[strings[0]].y - marker_dfs[strings[2]].y) + np.square(marker_dfs[strings[0]].z - marker_dfs[strings[2]].z))
#     line_dfs[lines[2]] = np.sqrt(np.square(marker_dfs[strings[1]].y - marker_dfs[strings[2]].y) + np.square(marker_dfs[strings[1]].z - marker_dfs[strings[2]].z))
    
#     slope_dfs['{}_{}'.format(0+i, 1+i)] = np.arctan((marker_dfs[strings[1]].z - marker_dfs[strings[0]].z) / (marker_dfs[strings[1]].y - marker_dfs[strings[0]].y))

#     angles_dfs['{}_{}_{}'.format(0 + i, 1 + i, 2 + i)] = np.arccos((line_dfs[lines[0]] + line_dfs[lines[1]] - line_dfs[lines[2]]) / (2 * line_dfs[lines[0]] * line_dfs[lines[1]]))

# for key in marker_dfs:
#     for i in range(7):
#        if int(key[-1]) == i:
# for l in line_dfs:
#     print(line_dfs[l])
    # print(line_dfs[l].describe()[1], line_dfs[l].describe()[2])

# for a in angles_dfs:
#     print(angles_dfs[a].describe()[1])

# for s in slope_dfs:
#     print(slope_dfs[s].describe()[1])

final_df = pd.DataFrame()
for i in range(5):
    df = pd.read_csv('{}_{}_{}.txt'.format(i, i+1, i+2))
    df['group'] = i

    strings = ['{}_{}'.format(i, 0+i), '{}_{}'.format(i, 1+i), '{}_{}'.format(i, 2+i)]
    marker_dfs[strings[0]] = df[df['index'] == 0].reset_index()
    marker_dfs[strings[1]] = df[df['index'] == 1].reset_index()
    marker_dfs[strings[2]] = df[df['index'] == 2].reset_index()

    lines = ['{}_{}'.format(0+i, 1+i), '{}_{}'.format(0+i, 2+i), '{}_{}'.format(1+i, 2+i)]
    # df[lines[0]] = np.sqrt(np.square(marker_dfs[strings[0]].y - marker_dfs[strings[1]].y) + np.square(marker_dfs[strings[0]].z - marker_dfs[strings[1]].z))
    # df[lines[1]] = np.sqrt(np.square(marker_dfs[strings[0]].y - marker_dfs[strings[2]].y) + np.square(marker_dfs[strings[0]].z - marker_dfs[strings[2]].z))
    # df[lines[2]] = np.sqrt(np.square(marker_dfs[strings[1]].y - marker_dfs[strings[2]].y) + np.square(marker_dfs[strings[1]].z - marker_dfs[strings[2]].z))
    line_dfs[lines[0]] = np.sqrt(np.square(marker_dfs[strings[0]].y - marker_dfs[strings[1]].y) + np.square(marker_dfs[strings[0]].z - marker_dfs[strings[1]].z))
    line_dfs[lines[1]] = np.sqrt(np.square(marker_dfs[strings[0]].y - marker_dfs[strings[2]].y) + np.square(marker_dfs[strings[0]].z - marker_dfs[strings[2]].z))
    line_dfs[lines[2]] = np.sqrt(np.square(marker_dfs[strings[1]].y - marker_dfs[strings[2]].y) + np.square(marker_dfs[strings[1]].z - marker_dfs[strings[2]].z))
    # print(len(line_dfs[lines[0]]), len(marker_dfs[strings[0]]))
    # print(line_dfs)
    # final_df = marker_dfs[strings[0]]
    # print(df)

    angles = ['{}_{}_{}'.format(0 + i, 1 + i, 2 + i), '{}_{}_{}'.format(1 + i, 2 + i, 0 + i), '{}_{}_{}'.format(2 + i, 0 + i, 1 + i)]
    angles_dfs[angles[0]] = np.arccos((line_dfs[lines[0]] + line_dfs[lines[1]] - line_dfs[lines[2]]) / (2 * line_dfs[lines[0]] * line_dfs[lines[1]]))
    angles_dfs[angles[1]] = np.arccos((line_dfs[lines[1]] + line_dfs[lines[2]] - line_dfs[lines[0]]) / (2 * line_dfs[lines[1]] * line_dfs[lines[2]]))
    angles_dfs[angles[2]] = np.arccos((line_dfs[lines[2]] + line_dfs[lines[0]] - line_dfs[lines[1]]) / (2 * line_dfs[lines[2]] * line_dfs[lines[0]]))
    
    # for j in range(3):
    df1 = pd.concat([line_dfs[lines[0]], line_dfs[lines[1]], line_dfs[lines[2]], angles_dfs[angles[0]], angles_dfs[angles[1]], angles_dfs[angles[2]]], axis=1)
    # print(len(df1))
    df1['group'] = i
    final_df = final_df.append(df1)
        # print(len(final_df))
    # final_df['group'] = i

# final_df = final_df.drop(['x', 'index', 'level_0'], axis=1)

# final_df = final_df.append(df)
final_df = final_df.sample(frac=1).reset_index(drop=True)
print(final_df)

# X = final_df[['y', 'z', 'yaw']].as_matrix()
X = final_df.drop(['group'], axis=1).as_matrix()
y = final_df['group'].as_matrix()

X_train, X_test, y_train, y_test = train_test_split(X, y)

# nn = MLPClassifier(hidden_layer_sizes=(10,10,10,10,10,10,10,10,10), activation='relu')
# nn = MLPClassifier(hidden_layer_sizes=(100,100,), activation='relu')
nn = MLPClassifier(hidden_layer_sizes=(40,40,40,40,40,40,), activation='relu')
nn.fit(X_train, y_train)
# pickle.dump( nn, open( "trained_nn.p", "wb" ))
y_nn = nn.predict(X_test)

print(metrics.f1_score(y_test, y_nn, average='weighted'))
print(metrics.confusion_matrix(y_test, y_nn))