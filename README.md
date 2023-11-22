*针对ubuntu20.04系统做了验证，gcc/g++版本默认分别为9和11，窗口环境为x11*

首先添加ROS环境变量，然后分别在第三方文件下编译好各个库

### 环境配置

1、打开ORB_SLAM/src文件夹下的ORBextractor.cc文件，添加两个头文件：

```cpp
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
```

2、打开ORB_SLAM/Thirdparty/g2o/g2o/solvers文件夹下的linear_solver_eigen.h文件，找到

```cpp
#51～56
class LinearSolverEigen: public LinearSolver<MatrixType>
{
   public:
     typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SparseMatrix;
     typedef Eigen::Triplet<double> Triplet;
     typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> PermutationMatrix;
```

3、打开ORB_SLAM文件夹下的CMakeLists.txt，添加以下内容：

```cmake
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
/usr/lib/x86_64-linux-gnu/libboost_system.so
/usr/lib/x86_64-linux-gnu/libboost_filesystem.so
)
```

4、修改manifest.xml

```xml
#删除
<depend package="opencv2"/>
```

5、解压Data文件夹中的词袋库文件

6、修改ORB_SLAM/src/MapPublisher.cc

```cpp
#31
    const char* MAP_FRAME_ID = "ORB_SLAM/World";
```

7、安装一些必要的库

```bash
sudo apt-get install libboost-all-dev
sudo apt-get install libsuitesparse-dev
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
sudo apt-get install libeigen3-dev
```

### 编译



### 运行