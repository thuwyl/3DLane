import numpy as np
import os

def get_calib_frome_file(calib_file):
    with open(calib_file) as f:
        lines = f.readlines()
    
    tmp = lines[2].strip().split(' ')[1:]
    P2 = np.array(tmp, dtype=np.float32)
    
    tmp = lines[4].strip().split(' ')[1:]
    R0 = np.array(tmp, dtype=np.float32)

    tmp = lines[5].strip().split(' ')[1:]
    Tr_velo_to_cam = np.array(tmp, dtype=np.float32)


    return {'P2': P2.reshape(3,4),
            'R0': R0.reshape(3,3),
            'Tr_velo_to_cam': Tr_velo_to_cam.reshape(3,4)}


class Calibration(object):
    def __init__(self, calib_file):
        if isinstance(calib_file, str):
            calib = get_calib_frome_file(calib_file)
        else:
            calib = calib_file
        self.P2 = calib['P2']    # 3 x 4
        self.R0 = calib['R0']    # 3 x 3
        self.V2C = calib_file['Tr_velo_to_cam']    # 3 x 4

        #camera intrinsics and extrinsics
        self.cu = self.P2[0, 2]
        self.cv = self.P2[1, 2]
        self.fu = self.P2[0, 0]
        self.fv = self.P2[1, 1]
        self.tx = self.P2[0, 3] / (-self.fu)
        self.ty = self.P2[1, 3] / (-self.fv)

    def cart_to_hom(self, pts):
        """
        input: pts  (N, 3) or (N, 2)
        output: pts_hom: (N, 4) or (N, 3)
        """
        pts_hom = np.hstack((pts, np.ones((pts.shape[0], 1), dtype=np.float32)))
        return pts_hom
    
    def lidar_to_rect(self, pts_lidar):
        """
        input: pts_lidar (N, 3)
        output: pts_rect (N, 3)

        transform lidar points from lidar coordinate to camera coordinate
        """
        pts_lidar_hom = self.cart_to_hom(pts_lidar)
        pts_rect = np.dot(pts_lidar_hom, np.dot(self.V2C.T, self.R0.T))
        return pts_rect
    
    def rect_to_img(self, pts_rect):
        """
        input: pts_rect (N, 3)
        out_put: pts_img (N, 2)
        project points in camera coordinate to the image plane
        """

        pts_rect_hom = self.cart_to_hom(pts_rect)
        pts_2d_hom = np.dot(pts_rect_hom, self.P2.T)
        pts_img = (pts_2d_hom[:, 0:2].T / pts_rect_hom[:, 2]).T    # (N, 2)
        pts_rect_depth = pts_2d_hom[:, 2] -self.P2.T[3,2]   # depth in rect camera coordinate
        return pts_img, pts_rect_depth


    def lidar_to_img(self, pts_lidar):
        """
        input: pts_lidar (N, 3)
        output: pts_img (N, 2)
        project lidar points to the image plane
        """
        pts_rect = self.lidar_to_rect(pts_lidar)
        pts_img, pts_rect_depth = self.rect_to_img(pts_rect)
        return pts_img, pts_rect_depth
