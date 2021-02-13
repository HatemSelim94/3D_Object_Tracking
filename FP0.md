## FP.1 Match 3D Objects
File name: camFusion_Student.cpp
Function name: matchBoundingBoxes
Line: 310
* A score matrix for every combination of bounding boxes in the previous and current frame is intilized with zeros.
* If matched keypoints are only enclosed by one bounding box in the previous and one in the current frame, the corresponding score of those two bounding boxes is increased by one. Lines:[316, 342]
* Each bounding box from the current fram is associated with a box from the previous frame based on the score(highest number of keypoint correspondences). lines:[358, 379]
* Returns a map of boxes ids.
# FP.2 Compute Lidar-based TTC
File name: camFusion_Student.cpp
Function name: computeTTCLidar
Line: 253
* For both the previous and the current frames:
    * Lidar points are sorted. lines:[267, 268]
    * The 1st(Q1) and 3rd(Q3) quantile are calculated. 
    * The points located inside the range [(Q1- 1.2 IQR) , (Q3 + 1.2 IQR)] are considered to be inliers. Where IQR is Q3 - Q1. lines[270, 277]
    * The closest point in the x-axis from the inliers points(minX) is selected for further TTC calculation. lines[279, 287]
* TTC is calculated according to the following formula:
TTC = minXCurr * (1/frameRate) / (minXPrev-minXCurr) . line: 303

# FP.3 Associate Keypoint Correspondences with Bounding Boxes
File name: camFusion_Student.cpp
Functon name: clusterKptMatchesWithROI
Line: 140
* For each match, the point from the current frame is checked if it is enclosed by the bounding box. 
* The match that corresponds to the enclosed point is pushed into a vector(candidateMatches) for further calculations. lines[145, 152]
* the distances between the matched key-points in a two successive frames is calculated. lines:[156, 162]
* Any distance lies outside the range  [(Q1- 1.2 IQR) , (Q3 + 1.2 IQR)], the match that corresponds to that distance is considered an outlier. 
* The inliers are pushed into the bounding box matches vector.[172, 178]

# FP.4 Compute Camera-based TTC
File name: camFusion_Student.cpp
Function name: computeTTCCamera
Line: 187
* To decrease the influence of any remaining outliers, the median(medianDistanceRatio) of all distance ratios dij/dij' is calculated, where i != j and " ' " represents the previous frame. lines: [194, 229]
* The TTC is calculated according to the following formula:
TTC = -(1/frameRate) / (1-medianDistanceRatio).  line: 239

# FP.5 Performance Evaluation 1
* A very small difference in distance of the chosen point to calculate the TTC makes a relatively big diffrence in the TTC. The manual estimation based TTC from the lidar top view is not error free. I think away from the outliers of the lidar points, a more complex model like the constant acceleration model would performed better.
 
# FP.6 Performance Evaluation 2
* All the cases where the camera based TTC is way off are indicated with red color in the spreadsheet(TTC Camera.pdf). 
* Potential reason: the majority of the keypoints are enclosed by the bounding box **but** they are not located on the preceding car or number of the keypoints detected is small.
* The combinations: SIFT/all , AKAZE / all, FAST/(BRIEF/BRISK) looks good based on the average per frame, and compared to the TTC Lidar and the manual TTC Lidar.


