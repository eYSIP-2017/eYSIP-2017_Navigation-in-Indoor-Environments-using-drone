from __future__ import print_function
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import StandardScaler
from sklearn import tree
from sklearn.svm import SVC
from sklearn import metrics
import itertools
import pickle

marker_dfs = dict()
line_dfs = dict()
angles_dfs = dict()
slope_dfs = dict()

def compute_parameters(a, b, c):
    ab = np.sqrt(np.square(a.y - b.y) + np.square(a.z - b.z))
    bc = np.sqrt(np.square(b.y - c.y) + np.square(b.z - c.z))
    ca = np.sqrt(np.square(a.y - c.y) + np.square(a.z - c.z))
    ab = ab.fillna(ab.mean())
    bc = bc.fillna(bc.mean())
    ca = ca.fillna(ca.mean())
    a_angle = np.arccos((ab + ca - bc) / (2 * ab * ca))
    b_angle = np.arccos((bc + ab - ca) / (2 * bc * ab))
    c_angle = np.arccos((ca + bc - ab) / (2 * ca * bc))
    # print((ab + ca - bc) / (2 * ab * ca))
    # print(len(ab), len(bc), len(ca))

    # compute distances from centroid
    centroid_y = (a.y + b.y + c.y)/3
    centroid_z = (a.z + b.z + c.z)/3
    cent_a = np.sqrt(np.square(a.y - centroid_y) + np.square(a.z - centroid_z))
    cent_b = np.sqrt(np.square(b.y - centroid_y) + np.square(b.z - centroid_z))
    cent_c = np.sqrt(np.square(c.y - centroid_y) + np.square(c.z - centroid_z))

    # get the absolute slope
    slope_ab = np.arctan(abs((a.z - b.z) / (a.y - b.y)))
    slope_bc = np.arctan(abs((b.z - c.z) / (b.y - c.y)))
    slope_ca = np.arctan(abs((c.z - a.z) / (c.y - a.y)))

    df = pd.concat([ab, bc, ca, a_angle, b_angle, c_angle, cent_a, cent_b, cent_c, slope_ab, slope_bc, slope_ca], axis=1)
    df = df.fillna(df.mean())
    return df

def compute_parameters_stepwise(a, b, c):
    ab = np.sqrt(np.square(a.y - b.y) + np.square(a.z - b.z))
    bc = np.sqrt(np.square(b.y - c.y) + np.square(b.z - c.z))
    ca = np.sqrt(np.square(a.y - c.y) + np.square(a.z - c.z))
    
    df = pd.concat([ab, bc, ca], axis=1)
    df = df.fillna(df.mean()).reset_index(drop=True)
    
    a_angle = np.arccos(((df[0] + df[2] - df[1]) / (2 * df[0] * df[2])))
    b_angle = np.arccos((df[1] + df[0] - df[2]) / (2 * df[1] * df[0]))
    c_angle = np.arccos((df[2] + df[1] - df[0]) / (2 * df[2] * df[1]))
    temp = ((df[0] + df[2] - df[1]) / (2 * df[0] * df[2]))
    print(temp.where(temp > 1).isnull().sum())
    # print(len(ab), len(bc), len(ca))
    return

    # compute distances from centroid
    centroid_y = (a.y + b.y + c.y)/3
    centroid_z = (a.z + b.z + c.z)/3
    cent_a = abs(np.sqrt(np.square(a.y - centroid_y) + np.square(a.z - centroid_z)))
    cent_b = abs(np.sqrt(np.square(b.y - centroid_y) + np.square(b.z - centroid_z)))
    cent_c = abs(np.sqrt(np.square(c.y - centroid_y) + np.square(c.z - centroid_z)))

    # get the absolute slope
    slope_ab = np.arctan((a.z - b.z) / (a.y - b.y))
    slope_bc = np.arctan((b.z - c.z) / (b.y - c.y))
    slope_ca = np.arctan((c.z - a.z) / (c.y - a.y))

    df = pd.concat([ab, bc, ca, a_angle, b_angle, c_angle, cent_a, cent_b, cent_c, slope_ab, slope_bc, slope_ca], axis=1)
    df = df.fillna(df.mean())
    return df

def compute_line_parameters(a, b):
    ab = np.sqrt(np.square(a.y - b.y) + np.square(a.z - b.z))

    # get the absolute slope
    slope = abs(np.arctan((a.z - b.z) / (a.y - b.y)))

    df = pd.concat([ab, slope], axis=1)
    df = df.fillna(df.mean())
    return df

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

def train_old():
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

def train_nn(final_df, store=False, filename="nn_train.p"):
    X = final_df.drop(['group'], axis=1).as_matrix()
    y = final_df['group'].as_matrix()

    X_train, X_test, y_train, y_test = train_test_split(X, y, random_state=3)

    scaler = StandardScaler()
    scaler.fit(X_train)
    X_train = scaler.transform(X_train)
    # apply same transformation to test data
    X_test = scaler.transform(X_test)

    # nn = MLPClassifier(hidden_layer_sizes=(10,10,10,10,10,10,10,10,10), activation='relu')
    # nn = MLPClassifier(hidden_layer_sizes=(100,100,), activation='relu')
    # nn = MLPClassifier(hidden_layer_sizes=(40,40,40,40,40,40,), activation='relu')
    nn = MLPClassifier(hidden_layer_sizes=(80, 80, 60, 60, 40, 40,), activation='relu', verbose=True)
    nn.fit(X_train, y_train)
    # pickle.dump( nn, open( "trained_nn.p", "wb" ))
    if store:
        pickle.dump( [nn, scaler], open( filename, "wb" ))
    y_nn = nn.predict(X_test)

    print(metrics.f1_score(y_test, y_nn, average='weighted'))
    print(metrics.confusion_matrix(y_test, y_nn))

def train_decision_tree(final_df, store=False, filename="decision_train.p"):
    X = final_df.drop(['group'], axis=1).as_matrix()
    y = final_df['group'].as_matrix()

    X_train, X_test, y_train, y_test = train_test_split(X, y, random_state=3)

    # scaler = StandardScaler()
    # scaler.fit(X_train)
    # X_train = scaler.transform(X_train)
    # # apply same transformation to test data
    # X_test = scaler.transform(X_test)

    clf = tree.DecisionTreeClassifier()
    clf = clf.fit(X_train, y_train)
    # pickle.dump( nn, open( "trained_nn.p", "wb" ))
    if store:
        pickle.dump(clf, open( filename, "wb" ))
    y_pred = clf.predict(X_test)

    print(metrics.f1_score(y_test, y_pred, average='weighted'))
    print(metrics.confusion_matrix(y_test, y_pred))

def train_with_all():
    final_df = pd.DataFrame()
    df = pd.read_csv('all.txt')
    df = df.append(pd.read_csv('old_all.txt'))
    df_list = list()
    for i in range(7):
        df_list.append(df[df['index'] == i].reset_index(drop=True))

    # fill_df(df_list[0], len(df_list[-1]))
    # fill_df(df_list[1], len(df_list[-1]))
    i = 0
    file = open('group.txt', 'w')
    file.truncate()
    try:
        for triangle in itertools.combinations(df_list, 3):
            # print(triangle[1])
            temp_df = compute_parameters(triangle[0], triangle[1], triangle[2])
            temp_df['group'] = i
            # triangle.append(i)
            file.write(str(triangle[0]['index'][0]) + ', ' + str(triangle[1]['index'][0]) + ', ' + str(triangle[2]['index'][0]) + ', ' + str(i) + '\n')
            # file.write(str(triangle))
            # file.write(', '+str(i))
            # file.write('\n')
            i += 1
            final_df = final_df.append(temp_df)
    finally:
        file.close()

    final_df = final_df.sample(frac=1).reset_index(drop=True)
    # train_nn(final_df)
    train_decision_tree(final_df)

def train_with_2_points():
    final_df = pd.DataFrame()
    df = pd.read_csv('old_all.txt')
    # df = df.append(pd.read_csv('old_all.txt'))
    df_list = list()
    for i in range(7):
        df_list.append(df[df['index'] == i].reset_index(drop=True))

    # fill_df(df_list[0], len(df_list[-1]))
    # fill_df(df_list[1], len(df_list[-1]))
    i = 0
    for lines in itertools.combinations(df_list, 2):
        # print(triangle[1])
        temp_df = compute_line_parameters(lines[0], lines[1])
        temp_df['group'] = i
        i += 1
        final_df = final_df.append(temp_df)

    final_df = final_df.sample(frac=1).reset_index(drop=True)
    train_nn(final_df)

def train_five_formation():
    final_df = pd.DataFrame()
    df = pd.read_csv('five_markers.txt')
    df_list = list()
    for i in range(7):
        df_list.append(df[df['index'] == i].reset_index(drop=True))
    i = 0
    file = open('five_group.txt', 'w')
    file.truncate()
    try:
        # for triangle in itertools.combinations(df_list, 3):
        #     # print(triangle[1])
        #     temp_df = compute_parameters(triangle[0], triangle[1], triangle[2])
        #     temp_df['group'] = i
        #     # triangle.append(i)
        #     file.write(str(triangle[0]['index'][0]) + ', ' + str(triangle[1]['index'][0]) + ', ' + str(triangle[2]['index'][0]) + ', ' + str(i) + '\n')
        #     i += 1
        #     final_df = final_df.append(temp_df)
        for lines in itertools.combinations(df_list, 2):
            temp_df = compute_line_parameters(lines[0], lines[1])
            temp_df['group'] = i
            # print(len(lines[0]))
            # print(lines)
            # file.write(str(lines[0]['index'][0]) + ', ' + str(lines[1]['index'][0]) + ', ' + str(i) + '\n')
            i += 1
            final_df = final_df.append(temp_df)
    finally:
        file.close()

    final_df = final_df.sample(frac=1).reset_index(drop=True)
    # train_nn(final_df)
    train_decision_tree(final_df)

if __name__ == '__main__':
    train_five_formation()