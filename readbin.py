import numpy as np
import open3d as o3d
from glob import glob
import os
import time

files = glob('D:/xuexi/pointcloud/data/002/velodyne_points/data/*.bin')



# p.points = o3d.utility.Vector3dVector(ps.T)
# p.colors = o3d.utility.Vector3dVector(cs)

#p.colors = o3d.utility.Vector3dVector(r[:, 3:] / 255)


v = o3d.visualization.Visualizer()
v.create_window()
p = o3d.geometry.PointCloud()
v.add_geometry(p)
First = True
for i,path in enumerate(files):
    (filepath, filename) = os.path.split(path)
    (name, suffix) = os.path.splitext(filename)

    points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    p.points = o3d.utility.Vector3dVector(points[:, :3])
    v.update_geometry(p)

    v.reset_view_point(First)
    First =False
    v.poll_events()
    v.update_renderer()
    print(i+1,'/',len(files))
    o3d.io.write_point_cloud(filepath + '/' + name+'.pcd', p)
    # time.sleep(0.3)
    #v.run()
v.destroy_window()

