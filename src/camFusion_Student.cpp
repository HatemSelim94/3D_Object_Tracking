
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));
    //static size_t frame = 0;
    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }
    //imwrite("../images/LidarTopView/"+to_string(frame++)+".JPEG", topviewImg);

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, cv::WINDOW_KEEPRATIO );
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ... train
    double mean = 0, distIQR, distQ1, distQ3;
    std::vector<cv::DMatch>candidateMatches; 
    for(auto match:kptMatches)
    {
        bool encolosed = boundingBox.roi.contains(kptsCurr[match.trainIdx].pt);
        if(encolosed)
        {
            candidateMatches.push_back(match);
        }
    }
    vector<double> kPointsMatchedDistances, sortedDistance;
    kPointsMatchedDistances.reserve(candidateMatches.size());
    sortedDistance.reserve(candidateMatches.size());
    for(auto match: candidateMatches)
    {
        cv::Point2f currPoint = kptsCurr[match.trainIdx].pt;
        cv::Point2f prevPoint = kptsPrev[match.queryIdx].pt;
        double distance = cv::norm(currPoint - prevPoint);
        kPointsMatchedDistances.push_back(distance);
    }
    sortedDistance = kPointsMatchedDistances;
    sort(sortedDistance.begin(), sortedDistance.end());

    
    distQ1 = sortedDistance[int(sortedDistance.size()*0.25)];
    distQ3 = sortedDistance[int(sortedDistance.size()*0.75)];  

    distIQR = distQ3 - distQ1;

    for(size_t i=0 ; i< kPointsMatchedDistances.size(); i++ )
    {
        if((kPointsMatchedDistances[i] > (distQ1- 1.2*distIQR)) && (kPointsMatchedDistances[i] < (distQ3 + 1.2*distIQR)) )
        {
            boundingBox.kptMatches.push_back(candidateMatches[i]);
            //boundingBox.keypoints.push_back(kptsCurr[kptMatches[i].trainIdx]);
        }
    }


    
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
    bool debug = true;
    double minDist = 100.0; // min. required distance
    vector<double> sortedDistRatios;
    for(size_t i=0; i < kptMatches.size(); i++) 
    {
        cv::Point2f currRefPoint = kptsCurr[kptMatches[i].trainIdx].pt;
        cv::Point2f prevRefPoint = kptsPrev[kptMatches[i].queryIdx].pt;
        for(size_t j = i+1; j < kptMatches.size();j++)
        {
            cv::Point2f currPoint = kptsCurr[kptMatches[j].trainIdx].pt;
            cv::Point2f prevPoint = kptsPrev[kptMatches[j].queryIdx].pt;
            double currDistance = cv::norm(currRefPoint - currPoint);
            double prevDistance = cv::norm(prevRefPoint - prevPoint);
             if (prevDistance > std::numeric_limits<double>::epsilon() && currDistance >= minDist)
            { // avoid division by zero

                double distRatio = currDistance / prevDistance;
                sortedDistRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (sortedDistRatios.size() != 0)
    {
        int medianIndex = sortedDistRatios.size()/2;
        double medianDistanceRatio;
        sort(sortedDistRatios.begin(), sortedDistRatios.end());
        
        if(sortedDistRatios.size()%2 != 0)
        {
            medianDistanceRatio = sortedDistRatios[medianIndex];
            if(debug)cout<<"Median Distance"<<medianDistanceRatio<<endl; 
        }
        else
        {
            medianDistanceRatio = (sortedDistRatios[medianIndex]+sortedDistRatios[medianIndex-1])/2;
            if(debug)cout<<"Median Distance"<<medianDistanceRatio<<endl; 
        }
        
       /* if(medianDistanceRatio <= 1)
        {
            // calculate the mean
            medianDistanceRatio = std::accumulate(sortedDistRatios.begin(), sortedDistRatios.end(), 0.0)/ sortedDistRatios.size();
            if(debug)cout<<"Mean Distance"<<medianDistanceRatio<<endl; 
        }*/
        

        TTC = -(1/frameRate) / (1 - medianDistanceRatio);   

    }
    else {
    
    TTC = NAN;
    
    }  
    

    //TTC = minXCurr * (1/frameRate) / (minXPrev-minXCurr);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
    double minXPrev = 0, minXCurr = 0, prevIQR, currIQR, currQ1, currQ3,prevQ1, prevQ3;
    vector<double> currSorted, prevSorted;
    for(size_t i=0; i<lidarPointsPrev.size(); i++)
    {
        prevSorted.push_back(lidarPointsPrev[i].x);
    }
    for(size_t i=0; i<lidarPointsCurr.size(); i++)
    {
        currSorted.push_back(lidarPointsCurr[i].x);
    }
    sort(currSorted.begin(),currSorted.end());
    sort(prevSorted.begin(),prevSorted.end());

    prevQ1 = prevSorted[int(prevSorted.size()*0.25)];
    prevQ3 = prevSorted[int(prevSorted.size()*0.75)];
    
    currQ1 = currSorted[int(currSorted.size()*0.25)];
    currQ3 = currSorted[int(currSorted.size()*0.75)];  

    prevIQR = prevQ3 - prevQ1;
    currIQR = currQ3 - currQ1;

    for(size_t i=0 ; minXPrev < ( prevQ1 - 1.2* prevIQR); i++ )
    {
        minXPrev = prevSorted[i];
    }

    for(size_t i=0 ; minXCurr < (currQ1 - 1.2* currIQR); i++ )
    {
        minXCurr = currSorted[i];
    }
    
    // Mean
    /*
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it)
    {
        minXPrev += it->x / lidarPointsPrev.size();
    }

    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it)
    {
        minXCurr += it->x / lidarPointsCurr.size();
    }

    */
    // compute TTC from both measurements
    TTC = minXCurr * (1/frameRate) / (minXPrev-minXCurr);
}





void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    bool debug = false;
    vector<vector<int>> scores(currFrame.boundingBoxes.size(),vector<int>(prevFrame.boundingBoxes.size(),0));
    // find the score of previous and current bounding boxes pair for all bounding boxes
    // for each bounding box in the previous scene find the corresponding one in the current frame based on the score
    for(auto match: matches)
    {
        vector<int> prevBBids;
        vector<int> currBBids;
        for(int i = 0 ; i < (prevFrame.boundingBoxes.size());i++)
        {
            bool bContained = prevFrame.boundingBoxes[i].roi.contains(prevFrame.keypoints[match.queryIdx].pt);
            if(bContained)
            {
                prevBBids.push_back(i);
            }
        }
        for(int i = 0 ; i < (currFrame.boundingBoxes.size());i++)
        {
            bool bContained = currFrame.boundingBoxes[i].roi.contains(currFrame.keypoints[match.trainIdx].pt);
            if(bContained)
            {
                currBBids.push_back(i);
            }
        }
        // skip the match if there is an overlap in the current or previous frame
        if(currBBids.size()==1 && prevBBids.size()==1)
        {
            scores[currBBids[0]][prevBBids[0]] +=1;
            //prevFrame.boundingBoxes[i].keypoints.push_back(prevFrame.keypoints[match.trainIdx]);
            //currFrame.boundingBoxes[j].keypoints.push_back(currFrame.keypoints[match.queryIdx]);
        }

    }
    // prev is the key in the map
    if(debug)
    {
        cout<<"scores:"<<endl;
        for ( const std::vector<int> &v : scores )
        {
            for ( int x : v ) std::cout << x << ' ';
            std::cout << std::endl;
        }
    }
      
    
    auto scoresModified = scores;
    for(size_t i=0; i < scores.size() ; i++)
    {
        int bestMatchid = std::max_element(scoresModified[i].begin(),scoresModified[i].end()) - scoresModified[i].begin();
        auto ret = bbBestMatches.insert({bestMatchid, i});
        while(ret.second == false && scoresModified[i][bestMatchid]>0 )
        {
            //cout<<i<<" "<<bestMatchid<<endl;
            
                scoresModified[i][bestMatchid] = 0;
                bestMatchid = std::max_element(scoresModified[i].begin(),scoresModified[i].end()) - scoresModified[i].begin();
                ret = bbBestMatches.insert({bestMatchid, i});
                //currFrame.boundingBoxes[i].trackID = bestMatchid;
                if(ret.second == false)
                {
                    if(scoresModified[i][bestMatchid]>scores[bbBestMatches.at(bestMatchid)][bestMatchid])
                    {
                        bbBestMatches[bestMatchid] = i;   
                        ret.second = true;
                    }
                }
        }
    }
    if(debug)
    {
        cout<<"modified scores:"<<endl;
        for ( const std::vector<int> &v : scoresModified )
        {
            for ( int x : v ) std::cout << x << ' ';
            std::cout << std::endl;
        } 
        cout<<"best matches"<<endl; 
        for (auto itr = bbBestMatches.begin(); itr != bbBestMatches.end(); ++itr) 
                    { 
                        cout << '\t' << itr->first << '\t' << itr->second << '\n'; 
                    } 
                cout << endl; 
 
    }
}
