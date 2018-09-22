
/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#include <libusb-1.0/libusb.h>
#include <libfreenect/libfreenect.hpp> // Re-written by Z.M.
#include "rgbd_openrobotics.hpp"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include<opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <System.h> // ORB-SLAM2 Library


using namespace std;



int main(int argc, char **argv)
{
    cv::Mat twc = cv::Mat::eye(4,4,CV_32F);

    // Initialize video stream from kinect
    Freenect::Freenect freenect;
    MyFreenectDevice* device;
    device = &freenect.createDevice<MyFreenectDevice>(0);
    device->startVideo();
    device->startDepth();
    
    if(argc != 3)
    {
        cerr << endl << "Usage: ./rgbd_kinect_freenect path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Retrieve paths to images
    int frameRate =  30 ; // Preferably recode to get from conf YAML file
    double sampleTime = 1/frameRate; 
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop

    int ni=0;
    for(;;)
    {
	++ni;
   	cv::Mat depthMat(640,480,CV_16UC1); // TODO : read params from file
    	cv::Mat depthf(640,480,CV_8UC1);
    	cv::Mat rgbMat = cv::Mat(640,480,CV_8UC3,cv::Scalar(0)); 
	
	try
	{
        // Get stream
	device->getVideo(rgbMat);
	device->getDepth(depthMat);
	}
	catch(std::exception const& e)
	{

   	cerr << "ERREUR While get device : " << e.what() << endl;

	}
	
	if( rgbMat.empty() || depthMat.empty())                      
    	{
       	cout <<  "Could not open or find the image" << std::endl ;
       	return -1;
        }	
	//depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0); // usless
	double tframe = sampleTime * ni;

// Chorno to wait if there is still time before next frame after SLAM ran
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
	try
	{	
        // Translation world to camera
	twc = SLAM.TrackRGBD(rgbMat,depthMat,tframe);
	
	}	
	catch(std::exception const& e)
	{

   	cerr << "ERREUR While Tracking " << e.what() << endl;

	}
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

       //Save Map and current position
	if (ni % frameRate == 0) 
        {  
	  SLAM.SaveMap("mapPoints.txt");
	}

        if (ni % frameRate == 0) 
        {  
	  string filename="currentPos.txt";
	  ofstream f;
          f.open(filename.c_str());
          f << fixed;
	  f << twc << endl; // Save transrotation world to camera 
	  f.close();
	}

	// Wait to load the next frame 
        if(ttrack<tframe)
            usleep((tframe-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();


    device->stopVideo();
    device->stopDepth();
    return 0;
}




