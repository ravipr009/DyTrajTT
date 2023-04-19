#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <cstdlib>
#include "opencv2/opencv.hpp"


using namespace std;
using namespace cv;

#define DEBUG

#include "ExtendedKF.h"
#include "Vision.h"
#include "Communication.h"
#include "Search.h"

int main(int argc, char** argv)
{

	//VideoWriter video;
	//video.open("hybrid.avi", CV_FOURCC('M','J','P','G'), 30, Size(COLS,ROWS), true);
	//ofstream data("thetadata.csv");
	Vision vision;
	//Client communication(argv[1], atoi(argv[2]));
	Search search;
	bool startprocess = false;
	bool hitdecision = false;
	unsigned short int Pos[3];
	char dmpnumber;
	search.LoadDataBase();
	vision.Initialize();


	bool status = true;
	Client pc(argv[1], atoi(argv[2]));
	cout<<"Connecting to server..."<<endl;
	status = pc.ConnectToServer();
	if(!status)
	{
		cout << "ERROR connecting to server... \n";
		return 0;
	}
	else
		cout<<"Connected to server..."<<endl;
	
	
	bzero(pc.Send_Data_Buffer, DATA_SIZE);


	namedWindow("Frame0");
	namedWindow("Frame1");
	//namedWindow("FrameHSV0");
	//namedWindow("FrameHSV1");
	namedWindow("FrameBIN0");
	namedWindow("FrameBIN1");

	char key = 'r';
	for(;;)
	{
		vision.GetBallPosition();
		vision.Filter(key, &startprocess);
		
		if(startprocess)
		{
			vision.Predict();
			vision.Decide(&hitdecision, &startprocess);
			//data<<atan(vision.Measurement0(0)/vision.Measurement0(1))<<","<<vision.EKF.StatePost.at<float>(0)<<","<<vision.EKF.StatePost.at<float>(1)<<","<<vision.FuturePosition0(0)<<","<<vision.FuturePosition0(1)<<endl;
			if(hitdecision)
			{
				Pos[0] = STRINGLENGTH*sin(vision.FuturePosition0.at<float>(0)) + CENTERX;
				Pos[2] = STRINGLENGTH*cos(vision.FuturePosition0.at<float>(0)) + CENTERY;
				Pos[1] = vision.FuturePosition1.at<float>(0);
				search.NearestDMP(Pos, &dmpnumber);
				pc.Communicate(dmpnumber);
			}
			
			//video<<vision.Frame[0];
		}
		
		vision.DisplayImages();
		key = 'r';
		key = waitKey(5);
		if(key == 'q')
			break;
		//if(key == 'h')
		//	data<<0<<","<<0<<","<<0<<","<<0<<","<<0<<endl;
		

	}
	//pc.Communicate(dmpnumber);
	//video.release();
	

	return 0;
}
