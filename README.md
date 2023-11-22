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

8、修改ORB_SLAM/include/ORBextractor.h

```cpp
#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
```

9、修改ORB_SLAM/include/PnPsolver.h

```cpp
#include <opencv2/opencv.hpp>
#include<opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include "MapPoint.h"
#include "Frame.h"
using namespace cv;
```

10、修改ORB_SLAM/src/ORBextractor.cc

```cpp
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
```

11、修改ORB_SLAM/src/PnPsolver.cc

```cpp
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include<opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include "Thirdparty/DBoW2/DUtils/Random.h"
#include <ros/ros.h>
#include <algorithm>

using namespace cv;
using namespace std;
```

12、修改ORB_SLAM/src/Sim3Solver.cc

```cpp
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>


#include <ros/ros.h>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"
using namespace cv;
```

13、修改Thirdparty/DBoW2/DBoW2/FClass.h

```cpp
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
```

14、修改Thirdparty/DBoW2/DBoW2/FORB.h

```cpp
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
```

15、修改Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h

```cpp
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <limits>
```

16、Thirdparty/DBoW2/CMakeLists.txt

```cpp
find_package(OpenCV 4.2 QUIET)
```

PS：如有遗漏参考仓库文件，此处不再赘述

### 编译

1. Make sure you have installed ROS and all library dependencies (boost, eigen3, opencv, blas, lapack).

2. Clone the repository:

   ```bash
   git clone https://github.com/raulmur/ORB_SLAM.git ORB_SLAM
   ```

3. Add the path where you cloned ORB-SLAM to the `ROS_PACKAGE_PATH` environment variable. To do this, modify your .bashrc and add at the bottom the following line (replace PATH_TO_PARENT_OF_ORB_SLAM):

   ```bash
   export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH_TO_PARENT_OF_ORB_SLAM
   ```

4. Build g2o. Go into `Thirdparty/g2o/` and execute:

   ```bash
   mkdir build
   cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make 
   ```

   *Tip: To achieve the best performance in your computer, set your favorite compilation flags in line 61 and 62 of* `Thirdparty/g2o/CMakeLists.txt` 
   	  (by default -03 -march=native)

5. Build DBoW2. Go into Thirdparty/DBoW2/ and execute:

   ```bash
   mkdir build
   cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make  
   ```

   *Tip: Set your favorite compilation flags in line 4 and 5 of* `Thirdparty/DBoW2/CMakeLists.txt` (by default -03 -march=native)

6. Build ORB_SLAM. In the ORB_SLAM root execute:

   **If you use ROS Indigo, remove the depency of opencv2 in the manifest.xml.**

   ```bash
   mkdir build
   cd build
   cmake .. -DROS_BUILD_TYPE=Release
   make
   ```

   *Tip: Set your favorite compilation flags in line 12 and 13 of* `./CMakeLists.txt` (by default -03 -march=native)

### 运行

1. Launch ORB-SLAM from the terminal (`roscore` should have been already executed):

   	rosrun ORB_SLAM ORB_SLAM PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE

  You have to provide the path to the ORB vocabulary and to the settings file. The paths must be absolute or relative   to the ORB_SLAM directory.  
  We already provide the vocabulary file we use in `ORB_SLAM/Data/ORBvoc.txt.tar.gz`. Uncompress the file, as it will be loaded much faster.

2. The last processed frame is published to the topic `/ORB_SLAM/Frame`. You can visualize it using `image_view`:

   	rosrun image_view image_view image:=/ORB_SLAM/Frame _autosize:=true

3. The map is published to the topic `/ORB_SLAM/Map`, the current camera pose and global world coordinate origin are sent through `/tf` in frames `/ORB_SLAM/Camera` and `/ORB_SLAM/World` respectively.  Run `rviz` to visualize the map:

   *in ROS Fuerte*:

   	rosrun rviz rviz -d Data/rviz.vcg

   *in ROS Groovy or a newer version*:

   	rosrun rviz rviz -d Data/rviz.rviz

4. ORB_SLAM will receive the images from the topic `/camera/image_raw`. You can now play your rosbag or start your camera node. 
   If you have a sequence with individual image files, you will need to generate a bag from them. We provide a tool to do that: https://github.com/raulmur/BagFromImages.

