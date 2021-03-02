import numpy as np
import utils.calibration as calibration
import cv2

# pts_2d_hom = np.array([[10,20,5],[140, 80, 20],[9,12,3]])
# pts_img = (pts_2d_hom[:, 0:2].T / pts_2d_hom[:, 2]).T 
# print(pts_img)


from lane import Lane

lane = Lane(root_dir='/home/wyl/Data/database/',voxel_range=[5.0, -30.0, 41.0, 30.0],
                    voxel_size=[0.05, 0.05])


# print(np.asarray([0,1,2,3]))
bev_map = lane.get_bev_bgr_np(1)

# print(type(bev_map))
cv2.imwrite('out.png', bev_map)