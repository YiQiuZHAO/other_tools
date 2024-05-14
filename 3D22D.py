import cv2
import numpy as np
import open3d as o3d

camera_par = np.asmatrix([[1.9506592935364870e+03, 0.0, 1.9770478473401959e+03, 0],
                          [0.0, 1.9508117738745232e+03, 1.0786204201895550e+03, 0],
                          [0.0, 0.0, 1.0, 0]])

D = np.asmatrix([-0.050519061674533024, -0.007992982752507883, 0.00970045657644595, -0.004354775040194558, 0.0])

t_word_to_cam = np.asmatrix(
    [[2.4747462378258280e-02, -9.9955232303502073e-01, -1.6839925611563663e-02, -9.2541271346932907e-02],
     [-1.3087302341509554e-02, 1.6519577885364300e-02, -9.9977861656954914e-01, 2.4302538338292576e+00],
     [9.9960858363250360e-01, 2.4962312041639460e-02, -1.2672595327247342e-02, -5.0924142692133323e+00],
     [0., 0., 0., 1]])

point_cloud = np.loadtxt('data/001/np.txt')
img = cv2.imread('data/001/2-2.jpg')
cols, crows = img.shape[:2]

# cv2.projectPoints()

x, y, z = point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2]
word = np.asarray([x, y, z, np.ones(x.shape)])

word_2 = np.asmatrix(word)

p = camera_par * t_word_to_cam
depth = np.zeros(img.shape[:2]).astype('float')
r = []

for point in word_2.T:
    point_r = p * point.T
    point_r = point_r.T
    point_r[0, 0] /= point_r[0, 2]
    point_r[0, 1] /= point_r[0, 2]
    if point_r[0, 0] >= 0 and point_r[0, 0] <= crows and 0 <= point_r[0, 1] <= cols and point_r[
        0, 2] >= 0:
        color = np.flip(img[int(point_r[0, 1]), int(point_r[0, 0])].reshape(1, -1))
        print('c')
    else:
        color = np.asarray([[0, 0, 0]])
    r.append(np.hstack((point[0,:3], color)))
    # depth[int(point_r[0, 1]), int(point_r[0, 0])] = point_r[0, 2]

r = np.asarray(r).reshape(-1, 6)
# r[:,0] /= 100
# r[:,1] /= 100
# np.savetxt('r.txt', r)
#
# depth2 = cv2.resize(depth,(0,0),fx=0.2,fy=0.2)
# color = cv2.resize(img,(0,0),fx=0.2,fy=0.2)
# xs = np.linspace(0,crows/5-1,int(crows/5))
# ys = np.linspace(0,cols/5-1,int(cols/5))
# X,Y = np.meshgrid(xs, ys)
# ps = np.asarray(X,Y,depth2)
#
# ps = np.array((np.where(depth2!=0)[0],np.where(depth2!=0)[1],depth2[depth2!=0]))
# cs = color[np.where(depth2!=0)].astype('float')/255

pp = o3d.geometry.PointCloud()
# p.points = o3d.utility.Vector3dVector(ps.T)
# p.colors = o3d.utility.Vector3dVector(cs)
pp.points = o3d.utility.Vector3dVector(r[:, :3])
pp.colors = o3d.utility.Vector3dVector(r[:, 3:] / 255)
v = o3d.visualization.Visualizer()
v.create_window()
v.add_geometry(pp)
v.poll_events()
v.update_renderer()
v.run()
v.destroy_window()
