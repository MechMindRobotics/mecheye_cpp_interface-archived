# Mech-Eye Interface
Official C++ interface for Mech-Eye cameras.

Please select the  proper branch corresponding to the camera firmware version.

## Installation

This part tells you how to configure and run sample projects on Windows.

#### Prerequisites

In order to use this interface you will need the following Prerequisites installed:
1. [Cmake](https://cmake.org/) -Version > 3.7 is required.
2. [Visual Studio](https://visualstudio.microsoft.com/) -Version > 2015 is recommended.
#### Using CMake to configure 

1. Clone the project and select project folder in Cmake. Select where you want to build.

   ![step1](./img/step1.jpg)

2. Click the configure and choose the compiler(Visual Studio).

   ![step2_2](./img/step2_2.jpg)

3. Extract files from **\Mech-Eye_interface\dependencies\dependencies for MechEye_OpenInterface.7z** and modify the path in the cmake according to the locations of these extracted files.

   ![step3](./img/step3.jpg)

4. Click **Generate** and then **Open Project** and project will be opened in Visual Studio.


## Quick Start

1. Open the Visual Studio.

2. Modify the IP address according to the actual address in every sample_xxx.cpp files and build it. Then VS will generate an .exe file in working directory, usually **.\build\Debug\MechEye_OpenInterface.exe**.

![ip](./img/ip.jpg)


3. Add all extrated .dll files from **dependencies for MechEye_OpenInterface.7z** to the working directory.

4. Run this .exe file and it will capture images and point clouds in the working directory.

## Project hierarchy

The following shows the hierarchy of project files

```
Mech-Eye_interface
├─ CMakeLists.txt
├─ README.md
├─ dependencies
│    └─ dependencies for MechEye_OpenInterface.7z
├─ img
│    ├─ ip.jpg
│    ├─ step.png
│    ├─ step1.jpg
│    ├─ step2.jpg
│    ├─ step2_2.jpg
│    └─ step3.jpg
├─ include
│    ├─ CameraClient.h
│    ├─ CameraCmd.h
│    ├─ PointCloudTools.h
│    ├─ ZmqClient.h
│    └─ sample.h
├─ json
│    ......
├─ sample
│    ├─ sample1_parameter.cpp
│    └─ sample2_ImgAndCloud.cpp
└─ src
       ├─ CameraClient.cpp
       ├─ PointCloudTools.cpp
       ├─ ZmqClient.cpp
       └─ main.cpp
```

* **dependencies**

  It contains all .dll and other dependencies needed by the project.

* **include**

  It contains all .h header files of interfaces.

* **json**

  Since C++ standard libraries don't contain any json libraries, this folder contains a third-party library for json.

* **sample**

  It contains 2 samples which briefly introduce the usage of interfaces.

* **src**

  It contains all source code, interface fucntions are defined in CameraClient.cpp. Main.cpp, sample1_parameter.cpp and sample2_ImgAndCloud are examples to show how to use interfaces.

## Brief Intro to interfaces

All interfaces and functions are in  **CameraClient.cpp**.

There are two main classes: CameraClient and ZmqClient. CameraClient is subclass of ZmqClient. You only need to focus on CameraClient.

* **CameraClient**

  * **connect()** : connect to the camera according to its ip address.

  * **captureDepthImg()** : capture a depth image and return it.

  * **captureColorImg()** : capture a color image and return it.

  * **getCameraIntri()**: get camera's intrinsic parameters.

  * **getCameraInfo()**: get camera's ip address.

  * **getCameraVersion()**: get camera's version number.

  * **getColorImgSize()** : get the height and width of the color image to be captured.
  
  * **getDepthImgSize()** : get the height and width of the depth image to be captured.
  
  * **getParameter()** : get the value of a specific parameter in camera.

  * **setParameter()** : set the value of a specific parameter in camera.

    **Attention**: Please be sure to know the meaning of your setting of parameters, **wrong setting could cause error!**

  * **captureRgbPointCloud()** : get a point cloud as pcl::PointXYZRGB


### Intro to samples

The original project provides 2 samples to show how to use interfaces. They are included under **.\sample**.

##### sample1_parameter.cpp

This sample shows how to set camera's paramters.

First, we need to know the actual IP address of camera and set it, and then connect:

```c++
CameraClient camera;
std::string error;
// Camera ip should be modified to actual IP address.
const std::string cameraIp = "192.168.3.146";
if (!camera.connect(cameraIp)) return -1; //return -1 if connection to camera fails

```

Then, we can get some brief info about camera:

```c++
std::cout 
	<< "Camera ID: " << camera.getCameraId() << std::endl
	<< "Version: " << camera.getCameraVersion() << std::endl 
	<< "Color Image Size: " << camera.getColorImgSize() << std::endl
	<< "Depth Image Size: " << camera.getDepthImgSize() << std::endl; 
```

Finally, we can set and get the value of a specific parameter, in this case, we choose exposure mode and time for 2D image:

```c++
std::cout << camera.setParameter("scan2dExposureMode",0) << std::endl;
std::cout << camera.getParameter("scan2dExposureMode", error) << std::endl;
std::cout << camera.setParameter("scan2dExposureTime", 20) << std::endl;
std::cout << camera.getParameter("scan2dExposureTime", error) << std::endl;
```

##### sample2_ImgAndCloud.cpp

This sample capture color images, depth images and point clouds and then save them on the disk.
