#include <Lane.h>

www::Lane::Lane(){};
www::Lane::~Lane(){};


void www::Lane::init(Eigen::Vector2f v_size, Eigen::Vector4f v_range){
    voxel_size = v_size;
    voxel_range = v_range;
    voxel_num(0) = int((voxel_range(2)-voxel_range(0))/voxel_size(0));
    voxel_num(1) = int((voxel_range(3)-voxel_range(1))/voxel_size(1));

    points = Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic>();
    image_bev = Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic>();
    points.resize(voxel_num(0),voxel_num(1));
    image_bev.resize(voxel_num(0),voxel_num(1));

}