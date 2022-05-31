from cProfile import label
import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import sys

import numpy as np
np.random.seed(1)

import argparse

args = sys.argv[1:]
parser = argparse.ArgumentParser()
parser.add_argument("-points", type=int, default=6)
ns = parser.parse_args(args)


np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

def clsq(A, dim):

    m, p = A.shape
    if p < dim+1:
        print('not enough unknowns')
        assert False

    if m < dim:
        print('not enough equations')
        assert False

    m = min(m, p)
    Q, R = np.linalg.qr(A)
    # extract only the relevant part of the decomposition
    R = R[0:m, 0:m]

    R2 = R[p-dim: m+1,p-dim: p+1]
    U,S,V = np.linalg.svd(R2)
    V = np.transpose(V)
    n = V[:, dim-1].reshape((dim,1))
    #n = np.abs(n) #-> check the signs we want

    #print(f'R= {R}')
    #print(f'R2= {R[p-dim: m+1,p-dim: p+1]}')
    #print(f'V= {V}')
    #print(f'S= {S}')

    aux1 = np.linalg.inv(R[0:p-dim, 0:p-dim])
    aux2 = R[0:p-dim, p-dim:p+1]
    c = -np.matmul(np.matmul(aux1, aux2), n)


    v = np.concatenate([c, n])
    #print(f"solution= {v}")
    error = np.absolute(np.matmul(A, v))
    error_mean = error.mean()
    #print(f"error mean: {error_mean}")

    return c, n

def fit_line():


    A = np.zeros((10,3))
    A[:, 0] = 1
    A[:,1] = 1.0*np.arange(1, 11)
    #A[:,2] = np.array([ 0.2, 1.0, 2.6, 3.6, 4.9, 5.3, 6.5, 7.8, 8.0, 9.0])
    A[:,2] = np.arange(1, 11)
    c, n = clsq(A,2)

    print(c)
    print(n)

    x = np.array(A[:, 1])
    y = -(float(c)+n[0]*x)/n[1]
    print(x)
    print(y)

    plt.scatter(A[:,1],  A[:,2])
    #plt.plot(A[:,1],  A[:,2], color="dimgray")
    plt.plot(x,  y, linestyle='--', color="dimgray")
    plt.axis('equal')

    #line1, = ax.plot(x, np.sin(x), '--', linewidth=2,

    plt.show()


    exit()


def get_line_points(c, n, x_min, x_max, y_min, y_max):

    c = float(c)
    nx = float(n[0])
    ny = float(n[1])

    if x_max - x_min > y_max - y_min:

        x1 = x_min
        x2 = x_max
        y1 = -(c+nx*x1)/ny
        y2 = -(c+nx*x1)/ny

    else:

        y1 = y_min
        y2 = y_max
        x1 = -(c+ny*y1)/nx
        x2 = -(c+ny*y2)/nx

    return np.array([x1, x2]), np.array([y1, y2])

def create_example_points():

    points_per_size = ns.points

    A = np.zeros((4*points_per_size,6))

    for i in range(0, points_per_size):
        A[0*points_per_size + i, 0] = 1.0
        A[1*points_per_size + i, 1] = 1.0
        A[2*points_per_size + i, 2] = 1.0
        A[3*points_per_size + i, 3] = 1.0

    min_t = 0.1
    max_t = 0.9

    points = np.zeros((points_per_size, 2))

    for i in range(0, points_per_size):
        t = min_t + (max_t - min_t) * (1.0 * i) / points_per_size

        points[i, 0] = t
        points[i, 1] = 1.0 - t

        #points[i, 0] = t
        #points[i, 1] = -1.0 + t
    noise_dev = 0.0 # 0.01

    points1 = np.array(points) + noise_dev*np.random.random((points_per_size, 2))
    points2 = np.array(points)
    points3 = np.array(points)
    points4 = np.array(points)

    points2[:,0] = points[:,0] + noise_dev*np.random.random(points_per_size)
    points2[:,1] = -points[:,1] + noise_dev*np.random.random(points_per_size)

    points3[:,0] = -points[:,0] + noise_dev*np.random.random(points_per_size)
    points3[:,1] = -points[:,1] + noise_dev*np.random.random(points_per_size)

    points4[:,0] = -points[:,0] + noise_dev*np.random.random(points_per_size)
    points4[:,1] = points[:,1] + noise_dev*np.random.random(points_per_size)

    return points1, points2 ,points3, points4


def prepare_input_matrix(input_points_1, input_points_2, input_points_3, input_points_4):

    total_points = input_points_1.shape[0] + input_points_2.shape[0] + input_points_3.shape[0] + input_points_4.shape[0]
    num_points_1 = input_points_1.shape[0]
    num_points_2 = input_points_2.shape[0]
    num_points_3 = input_points_3.shape[0]
    num_points_4 = input_points_4.shape[0]

    index1 = num_points_1
    index2 = index1 + num_points_2
    index3 = index2 + num_points_3
    index4 = index3 + num_points_4

    A = np.zeros((total_points, 6), dtype=np.float64)
    b = np.zeros((total_points, 1), dtype=np.float64)

    A[0: index1, 0] = 1
    A[index1: index2, 1] = 1
    A[index2: index3, 2] = 1
    A[index3: index4, 3] = 1

    A[0: index1, 4] = input_points_1[:,0]
    A[0: index1, 5] = input_points_1[:,1]
    A[index1: index2, 4] = input_points_2[:,1]
    A[index1: index2, 5] = -input_points_2[:,0]
    A[index2: index3, 4] = input_points_3[:,0]
    A[index2: index3, 5] = input_points_3[:,1]
    A[index3: index4, 4] = input_points_4[:,1]
    A[index3: index4, 5] = -input_points_4[:,0]

    return A

def calculate_model_error(c, n, points, max_error):

    points = points[:,0:2]

    max_t_12 = np.linalg.norm(c[1] - c[0])
    max_t_23 = np.linalg.norm(c[2] - c[1])
    max_t_34 = np.linalg.norm(c[3] - c[2])
    max_t_41 = np.linalg.norm(c[0] - c[3])

    n = np.reshape(n, (2,))
    n1 = np.array(n)
    n2 = np.array(n)
    n2[0], n2[1] = n[1], -n2[0]
    n3 = np.array(n)
    n4 = np.array(n2)

    c1, c2, c3, c4 = c[0], c[1], c[2], c[3]

    points_number = points.shape[0]

    error_sum = 0.0
    inliers_1 = []
    inliers_2 = []
    inliers_3 = []
    inliers_4 = []

    for i in range(points_number):
        p = points[i]

        t1 = np.dot(p - c1, n1)
        t1 = min(1.0, max(0.0, max_t_12))
        d1 = c1 + t1*n1
        #error_1 = np.linalg.norm(p - d1)
        error_1 = abs(np.dot(p,n1) + c1)

        t2 = np.dot(p - c2, n2)
        t2 = min(1.0, max(0.0, max_t_23))
        d2 = c2 + t2*n2
        #error_2 = np.linalg.norm(p - d2)
        error_2 = abs(np.dot(p,n2) + c2)

        t3 = np.dot(p - c3, n3)
        t3 = min(1.0, max(0.0, max_t_34))
        d3 = c3 + t3*n3
        #error_3 = np.linalg.norm(p - d3)
        error_3 = abs(np.dot(p,n3) + c3)

        t4 = np.dot(p - c4, n4)
        t4 = min(1.0, max(0.0, max_t_41))
        d4 = c4 + t4*n4
        #error_4 = np.linalg.norm(p - d4)
        error_4 = abs(np.dot(p,n4) + c4)

        error = min(error_1, error_2, error_3, error_4)
        error_sum += error

        if error_1 < max_error:
            inliers_1.append(p)
        if error_2 < max_error:
            inliers_2.append(p)
        if error_3 < max_error:
            inliers_3.append(p)
        if error_4 < max_error:
            inliers_4.append(p)

    return error_sum / points_number, np.array(inliers_1), np.array(inliers_2), np.array(inliers_3), np.array(inliers_4)

def rectangle_ransac(input_points_1, input_points_2, input_points_3, input_points_4):

    min_points = 100
    max_iterations = 100
    max_error = 0.01

    num_points_1 = input_points_1.shape[0]
    num_points_2 = input_points_2.shape[0]
    num_points_3 = input_points_3.shape[0]
    num_points_4 = input_points_4.shape[0]

    points = np.concatenate([input_points_1, input_points_2, input_points_3, input_points_4])

    min_error = 1e9
    best_c = None
    best_n = None

    inliers_1 = None
    inliers_2 = None
    inliers_3 = None
    inliers_4 = None

    for iteration in range(0, max_iterations):

        # Sample the points for the hypothesis
        indexes_1 = np.random.randint(0, high=num_points_1, size=[min_points])
        indexes_2 = np.random.randint(0, high=num_points_2, size=[min_points])
        indexes_3 = np.random.randint(0, high=num_points_3, size=[min_points])
        indexes_4 = np.random.randint(0, high=num_points_4, size=[min_points])

        tmp_points_1 = input_points_1[indexes_1]
        tmp_points_2 = input_points_2[indexes_2]
        tmp_points_3 = input_points_3[indexes_3]
        tmp_points_4 = input_points_4[indexes_4]

        #tmp_points_1 = input_points_1
        #tmp_points_2 = input_points_2
        #tmp_points_3 = input_points_3
        #tmp_points_4 = input_points_4

        A = prepare_input_matrix(tmp_points_1, tmp_points_2, tmp_points_3, tmp_points_4)

        c, n = clsq(A,2)
        #print(f'c={c}')
        #print(f'n={n}')

        error, tmp_inliers_1, tmp_inliers_2, tmp_inliers_3, tmp_inliers_4 = calculate_model_error(c, n, points, max_error)

        if error < min_error:
            min_error = error
            print(f"Iteration {iteration} -> Error: {error}")
            best_c = c
            best_n = n

            inliers_1 = tmp_inliers_1
            inliers_2 = tmp_inliers_2
            inliers_3 = tmp_inliers_3
            inliers_4 = tmp_inliers_4

    #A = prepare_input_matrix(inliers_1, inliers_2, inliers_3, inliers_4)
    #c, n = clsq(A,2)

    return c, n, inliers_1, inliers_2, inliers_3, inliers_4




#fit_line()

cluster_index = 68

corners = o3d.io.read_point_cloud(f"data/{cluster_index}_corners.pcd")
cloud1 = o3d.io.read_point_cloud(f"data/{cluster_index}_cloud1.pcd")
cloud2 = o3d.io.read_point_cloud(f"data/{cluster_index}_cloud2.pcd")
cloud3 = o3d.io.read_point_cloud(f"data/{cluster_index}_cloud3.pcd")
cloud4 = o3d.io.read_point_cloud(f"data/{cluster_index}_cloud4.pcd")


corners_points = np.asarray(corners.points)
cloud1_points = np.asarray(cloud1.points)
cloud2_points = np.asarray(cloud2.points)
cloud3_points = np.asarray(cloud3.points)
cloud4_points = np.asarray(cloud4.points)

input_points_1 = cloud1_points
input_points_2 = cloud2_points
input_points_3 = cloud3_points
input_points_4 = cloud4_points


example_points1, example_points2, example_points3, example_points4 = create_example_points()

#input_points_1 = example_points1
#input_points_2 = example_points2
#input_points_3 = example_points3
#input_points_4 = example_points4

x_min = min([input_points_1[:,0].min(), input_points_2[:,0].min(), input_points_3[:,0].min(), input_points_4[:,0].min()])
x_max = max([input_points_1[:,0].max(), input_points_2[:,0].max(), input_points_3[:,0].max(), input_points_4[:,0].max()])

y_min = min([input_points_1[:,1].min(), input_points_2[:,1].min(), input_points_3[:,1].min(), input_points_4[:,1].min()])
y_max = max([input_points_1[:,1].max(), input_points_2[:,1].max(), input_points_3[:,1].max(), input_points_4[:,1].max()])

dx = x_max - x_min
dy = y_max - y_min

#margin = 0.5
margin = 0.5
x_min -= margin*dx
x_max += margin*dx

y_min -= margin*dy
y_max += margin*dy

A = prepare_input_matrix(input_points_1, input_points_2, input_points_3, input_points_4)

c, n = clsq(A,2)

inliers_1, inliers_2, inliers_3, inliers_4 = None, None, None, None
#c, n, inliers_1, inliers_2, inliers_3, inliers_4 = rectangle_ransac(input_points_1, input_points_2, input_points_3, input_points_4)
print(f'c={c}')
print(f'n={n}')

#plt.scatter(A[:, 4], A[:, 5], s=150, label="test")

#plt.scatter(corners_points[:,0], corners_points[:,1], s=150, label="RANSAC corners")
plt.scatter(input_points_1[:,0], input_points_1[:,1], label="cloud 1")
plt.scatter(input_points_2[:,0], input_points_2[:,1], label="cloud 2")
plt.scatter(input_points_3[:,0], input_points_3[:,1], label="cloud 3")
plt.scatter(input_points_4[:,0], input_points_4[:,1], label="cloud 4")

if inliers_1 is not None and inliers_1.shape[0] > 0:
    plt.scatter(inliers_1[:,0], inliers_1[:,1], label="inliers 1", marker='x', s=150)
if inliers_2 is not None and inliers_2.shape[0] > 0:
    plt.scatter(inliers_2[:,0], inliers_2[:,1], label="inliers 2", marker='x', s=150)
if inliers_3 is not None and inliers_3.shape[0] > 0:
    plt.scatter(inliers_3[:,0], inliers_3[:,1], label="inliers 3", marker='x', s=150)
if inliers_4 is not None and inliers_4.shape[0] > 0:
    plt.scatter(inliers_4[:,0], inliers_4[:,1], label="inliers 4", marker='x', s=150)

plt.plot([5*x_min, 5*x_max], [0.0, 0.0], linestyle='--', color="dimgray")
plt.plot([0.0, 0.0], [5*y_min, 5*y_max], linestyle='--', color="dimgray")
plt.plot(list(corners_points[:,0]) + [corners_points[0,0]], list(corners_points[:,1]) + [corners_points[0,1]], linestyle='--', color="dimgray")


cloud_1_fitted_lines = get_line_points(c[0], [n[0], n[1]], x_min, x_max, y_min, y_max)
cloud_2_fitted_lines = get_line_points(c[1], [-n[1], n[0]], x_min, x_max, y_min, y_max)
cloud_3_fitted_lines = get_line_points(c[2], [n[0], n[1]], x_min, x_max, y_min, y_max)
cloud_4_fitted_lines = get_line_points(c[3], [-n[1], n[0]], x_min, x_max, y_min, y_max)

plt.plot(cloud_1_fitted_lines[0], cloud_1_fitted_lines[1], linestyle='--', label="line1")
plt.plot(cloud_2_fitted_lines[0], cloud_2_fitted_lines[1], linestyle='--', label="line2") # abajo derecha
plt.plot(cloud_3_fitted_lines[0], cloud_3_fitted_lines[1], linestyle='--', label="line3") # abajo derecha
plt.plot(cloud_4_fitted_lines[0], cloud_4_fitted_lines[1], linestyle='--', label="line4") # arriba izq


plt.axis('equal')

#line1, = ax.plot(x, np.sin(x), '--', linewidth=2,

ax = plt.gca()
ax.set_xlim([x_min, x_max])
ax.set_ylim([y_min, y_max])
ax.legend()


plt.show()
#plt.savefig("line_ransac.png")
