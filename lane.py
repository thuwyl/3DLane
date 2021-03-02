from kitti_dataset import KITTIDataset
import numpy as np
from numba import jit

class Lane(KITTIDataset):
    def __init__(self, root_dir, split='train', 
                    voxel_range=[0.0, -25.0, 80.0, 25.0],
                    voxel_size=[0.1, 0.1]):
        super().__init__(root_dir=root_dir, split=split)
        
        self.voxel_range = np.array(voxel_range)    # z_min x_min z_max x_max
        self.voxel_size = np.array(voxel_size)    # dz dx
        self.voxel_num = ((self.voxel_range[2:4] - self.voxel_range[0:2])/self.voxel_size).astype(int)    # x y
        
        
        # all bev maps are in camera coordinate 
        # self.bev_bgr_np = np.zeros((self.voxel_num[0], self.voxel_num[1],3))
        # self.bev_xyz_np = np.zeros((self.voxel_num[0], self.voxel_num[1],3))


    def get_bev_xyz_np(self, idx):
        '''
        output: xyz (N, 3) coordinates of all voxels in camera coordinate
        '''
        ids = np.arange(0, self.voxel_num[0]*self.voxel_num[1])
        ids = np.array([np.floor(ids/self.voxel_num[1]), ids % self.voxel_num[1]]).T
        
        zx = ids*np.array([-self.voxel_size[0], self.voxel_size[1]]) + np.array([self.voxel_range[2], self.voxel_range[1]])

        plane = self.get_road_plane(idx)
        y = -(zx[:,1:2] * plane[0] + zx[:,0:1] * plane[2] + plane[3])/plane[1]
        xyz = np.concatenate((zx[:,1:2], y, zx[:,0:1]), axis=1)
        return xyz.reshape((self.voxel_num[0], self.voxel_num[1],3))

    def color_points(self, idx, pts_rect):
        """
        input: pts_rect  (N, 3)   in camera coordinate
        output: color (N, 3)
        """
        calib = self.get_calib(idx)
        img = self.get_image(idx)
        h, w, _ = self.get_image_shape(idx)
        pts_img,_ = calib.rect_to_img(pts_rect)
        pts_color = np.zeros((len(pts_img[:,0]), 3))

        for i in range(len(pts_img[:,0])):
            if round(pts_img[i, 0]) >= 0 and round(pts_img[i, 0]) < w and  round(pts_img[i, 1]) >= 0 and round(pts_img[i, 1]) < h:
                bgr = img[round(pts_img[i, 1]), round(pts_img[i, 0]), :].tolist()
                pts_color[i,:] = bgr
        return pts_color


    def get_bev_bgr_np(self, idx):
        '''
        output: pts_color (N,3)
        '''
        pts_rect = self.get_bev_xyz_np(idx).reshape((self.voxel_num[0]*self.voxel_num[1],3))
        pts_color = self.color_points(idx, pts_rect)
        return pts_color.reshape((self.voxel_num[0], self.voxel_num[1],3))


    

