import numpy as np
import utils.calibration as calibration

# pts_2d_hom = np.array([[10,20,5],[140, 80, 20],[9,12,3]])
# pts_img = (pts_2d_hom[:, 0:2].T / pts_2d_hom[:, 2]).T 
# print(pts_img)


from lane import Lane

lane = Lane(root_dir='~',voxel_range=[0.0, -25.0, -3.0, 80.0, 25.0, 1.0],
                    voxel_size=[0.2, 0.2, 0.6])

print(lane.voxel_num)