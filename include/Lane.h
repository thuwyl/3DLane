#include <iostream>
#include <Eigen/Dense>

namespace www{
class Lane{
public:
    Lane();
    ~Lane();
    void init(Eigen::Vector2f v_size, Eigen::Vector4f v_range);

    Eigen::Vector2f voxel_size; //dx dy
    Eigen::Vector4f voxel_range; //x_min, y_min, x_max, y_max
    Eigen::Vector2i voxel_num; //x, y
    Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic> points, image_bev;
    


    
};
}

