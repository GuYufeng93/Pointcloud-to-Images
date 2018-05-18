# PointCloud to Images

An algorithm for projecting three-dimensional laser point cloud data into serialized two-dimensional images.  
Author:Yufeng Gu Guyufeng@mail.dlut.edu.cn

## Introduce

A viewpoint is selected on the center of the point cloud data or on the collection trajectory of the data. The algorithm then projects the 3D point cloud data onto the plane corresponding to the different view angles with the viewpoint as the center. The image is then dyed using the characteristics of a three-dimensional laser point cloud. This algorithm gives a total of six kinds of staining methods, the reader can choose one or more of them according to the need.
![algorithm_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/algorithm.png)  

The six staining methods are: RGB color, reflection value, vertical component of the normal vector, depth, bearing angle, and neighborhood space angle images.The number of generated pictures and the size and resolution of the generated picture can be changed.

### 1 serialized bearing angle images
![ETH BA_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/ETH_BA.gif)
![ETH BA_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/JACOBS_BA.gif)  

### 2 serialized depth images
![ETH Depth_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/ETH_Depth.gif)
![ETH Depth_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/JACOBS_Depth.gif)  

### 3 serialized intensity images
![ETH Intensity_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/ETH_Intensity.gif)
![ETH Intensity_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/JACOBS_Intensity.gif)  

### 4 serialized normal images
![ETH Normal_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/ETH_Normal.gif)
![ETH Normal_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/JACOBS_Normal.gif)  

### 5 serialized neighborhood space angle images
![ETH NSA_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/ETH_NSA.gif)
![ETH NSA_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/JACOBS_NSA.gif)  

### 6 serialized binary neighborhood space angle images
![ETH BNSA_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/ETH_BNSA.gif)
![ETH BNSA_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/JACOBS_BNSA.gif)  

### 7 serialized RGB color images
![ETH RGB color_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/ETH_RGB.gif)  

## Dependence

Program Dependency: PCL1.8.0 , OpenCV 3 , OpenMP.  
Regardless of the input and output, it takes 4 to 5 seconds to generate the six 360*360 images in 60 views. The number of three-dimensional laser point clouds is 8 million and the experimental platform is a notebook equipped with a Visual Studio 2015 development environment.

## Test data

### 1 Semantic3d:"bildstein3"  
http://www.semantic3d.net/view_dbase.php?chl=1 
### 2 Jacobs University Bremen gGmbH:"scan004"  
http://kos.informatik.uni-osnabrueck.de/3Dscans/ 