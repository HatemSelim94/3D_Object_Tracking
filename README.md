#### 3D Object Tracking
In this project, obstacles were detected and tracked with a camera and lidar sensors mounted on a vehicle.

##### Images:
* Image keypoints and keypoints descriptors are extracted using the best of detector+ descriptor combination based on performance analysis.
* Keypoints are matched across two consecutive frames.
* Images are processed through YOLO as an out-of-the-box obstacle detector providing a bounding box dimension and the center of the detected obstacle.
##### Lidar Points:
* Lidar points are cropped to focus on the preceding vehicle.
* Then, Lidar points are clustered by associating lidar points enclosed by a bounding box to that box.

Finally, 3d objects are matched across successive frames, and the TTC (time to collision) is calculated based on camera and lidar data.
For simplicity, the TTC estimation is done only for the preceding vehicle.
___
* To install dependencies, follow the instructions provided [here](https://github.com/HatemSelim94/3D_Object_Tracking/blob/main/Udacity_README%20.md).
* Find more details about implementation * [here](https://github.com/HatemSelim94/3D_Object_Tracking/blob/main/FP0.md).
* In [LidarTTC](https://github.com/HatemSelim94/3D_Object_Tracking/blob/main/LidarTTC%20.pdf) you can find the performance analysis of the TTC Lidar on a frame-by-frame basis, although the manual calculation is not error-free.
* In [Camera TTC](https://github.com/HatemSelim94/3D_Object_Tracking/blob/main/CameraTTC%20.pdf) performance analysis on a frame-by-frame basis can be found. Red cells represent a poor estimation of the TTC compared to the TTC frame average. 
* Camera images with TTC estimation are located [here](https://github.com/HatemSelim94/3D_Object_Tracking/tree/main/images/TTC).
