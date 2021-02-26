#include <Lane.h>

int main(){
    www::Lane Lane1;

    Eigen::Vector2f v_size(0.1, 0.1);
    Eigen::Vector4f v_range(0.0, -25.0, 50.0, 30.0);
    Lane1.init(v_size,v_range);
    return 1;
}