# PointCloud to Images

An algorithm for projecting three-dimensional laser point cloud data into serialized two-dimensional images.

## Introduce

A viewpoint is selected on the center of the point cloud data or on the collection trajectory of the data. The algorithm then projects the 3D point cloud data onto the plane corresponding to the different view angles with the viewpoint as the center. The image is then dyed using the characteristics of a three-dimensional laser point cloud. This algorithm gives a total of six kinds of staining methods, the reader can choose one or more of them according to the need.
![algorithm_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/algorithm.png)
algorithm

The six staining methods are: RGB color, reflection value, vertical component of the normal vector, depth, bearing angle, and a binarized grayscale image (no specific name yet).The number of generated pictures and the size and resolution of the generated picture can be changed.

### 1 BA_image
![BA_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/Bearing%20angle.gif)

### 2 Depth_image
![Depth_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/Depth.gif)

### 3 Intensity_image
![Intensity_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/I.gif)

### 4 Normal_image
![Normal_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/N.gif)

### 5 RGB color_image
![RGB color_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/RGB.gif)

### 6 Unnamed binarized image
![Unnamed_image](https://github.com/GuYufeng93/Pointcloud-to-Images/blob/master/Examples/PBA.gif)


## Dependence

Program Dependency: PCL1.8.0 , OpenCV 3 , OpenMP.  
Regardless of the input and output, it takes 4 to 5 seconds to generate the six 360*360 images in 60 views. The number of three-dimensional laser point clouds is 8 million and the experimental platform is a notebook equipped with a Visual Studio 2015 development environment.

## Test data

Semantic3d:"bildstein3"  
http://www.semantic3d.net/view_dbase.php?chl=1 