# lidarForPosition
------
## 第一次拉下仓库需要做的事情
### 1、编译安装雷达的底层库
首先在克隆下的仓库找到**urg_library-1.2.5**文件夹，打开终端，进入该路径(xxx/lidarForPosition/urg_library-1.2.5)下，运行以下指令
> * sudo make
> * sudo make install

### 2、编译整个工程
> 如果不先安装雷达的库的话，工程无法编译成功。

打开终端，进入克隆下的仓库的路径(xxx/lidarForPosition)，运行以下指令
> * mkdir build && cd build
> * cmake ..
> * make -j4

## 关于报错
### 1、#include <opencv/cv.h>未找到
由于在opencv3中opencv2的cv.h融合进了imgproc.hpp里，所以把代码中的#include <opencv/cv.h>改成#include <opencv2/imgproc.hpp>即可。