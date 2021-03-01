from kitti_dataset import KITTIDataset
import numpy as np

class Lane(KITTIDataset):
    def __init__(self, root_dir, split='train', 
                    voxel_range=[0.0, -25.0, -3.0, 80.0, 25.0, 1.0],
                    voxel_size=[0.1, 0.2, 0.5]):
        super().__init__(root_dir=root_dir, split=split)
        
        self.voxel_range = np.array(voxel_range)
        self.voxel_size = np.array(voxel_size)
        self.voxel_num = ((self.voxel_range[3:6] - self.voxel_range[0:3])/self.voxel_size).astype(int)
        
        