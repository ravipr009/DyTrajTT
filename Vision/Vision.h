#ifndef Vision_H_
#define Vision_H_


#define CAMERA0 1
#define CAMERA1 0

#define ROWS 480
#define COLS 640



class Vision
{
	public:
		Mat Frame[2], FrameHSV[2], FrameBIN[2];
		VideoCapture Camera[2];
		unsigned int Position[2][2];
		unsigned int WhiteCount[2];
		unsigned short int FilterParameters[2][6];
		KalmanFilter KF;
		ExtendedKalmanFilter EKF;
		Mat_<float> FuturePosition0;
		Mat_<float> Measurement0;
		Mat_<float> FuturePosition1;
		Mat_<float> Measurement1;
		char IsBall1;
		void Initialize();
		void GetBallPosition();
		void Filter(char, bool*);
		void Predict();
		void DisplayImages();
		void Decide(bool*, bool*);
};

void Vision::Initialize()
{
	unsigned short int i, j;

	Camera[0].open(CAMERA0);
	if(!Camera[0].isOpened())  // check if we succeeded
    	{
        	cout<<" Camera 0 is not Opened..."<<endl;
            		exit(0);
    	}
	
	Camera[1].open(CAMERA1);
	if(!Camera[1].isOpened())  // check if we succeeded
    	{
        	cout<<" Camera 1 is not Opened..."<<endl;
            		exit(0); 
    	}
	
	ifstream inputfile("config.csv");
	string line;
	if(inputfile.good())
	{

		for(i=0; i<2; i++)
		{
			cout<<" Filter "<<i<<" Parameters ";
			for(j=0; j<6; j++)
			{
				getline (inputfile, line, ',');
				FilterParameters[i][j] = atoi(line.c_str());
				cout<<FilterParameters[i][j]<<", ";
			}
			cout<<endl;
		}
		/*
		getline (inputfile, line, ',');
		Gravity = atof(line.c_str());
		cout<<" Gravity "<<Gravity<<endl;
		getline (inputfile, line, ',');
		Friction = atof(line.c_str());
		cout<<" Friction "<<Friction<<endl;
		*/
	}
	else
	{
		cout<<" Cannot find config.csv"<<endl;
		exit(0);
	}

	//KF.init(6, 2, 0);

	
	KF.init(4, 2, 0);
	KF.statePost.setTo(Scalar(0));
	
	Measurement1.create(2, 1);
	Measurement1.setTo(Scalar(0));
	FuturePosition1.create(2, 1);
	FuturePosition1.setTo(Scalar(0));

	/*
	KF.transitionMatrix = *(Mat_<float>(6, 6) << 	1, 0, DT, 0, 0.5*DT*DT, 0,   
							0, 1, 0, DT, 0, 0.5*DT*DT,  
							0, 0, 1, 0, DT, 0,  
							0, 0, 0, 1, 0, DT,
							0, 0, 0, 0, 1, 0,
							0, 0, 0, 0, 0, 1);
	*/
	KF.transitionMatrix = *(Mat_<float>(4, 4) << 	1, 0, DT, 0,   
							0, 1, 0, DT,  
							0, 0, 1, 0,  
							0, 0, 0, 1);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1));
	setIdentity(KF.measurementNoiseCov, Scalar::all(0.01));
	setIdentity(KF.errorCovPost, Scalar::all(10));


	EKF.Initialization(2, 2, 0);
	setIdentity(EKF.ProcessNoiseCov, Scalar::all(0.49));
	setIdentity(EKF.MeasurementNoiseCov, Scalar::all(0.01));
	setIdentity(EKF.ErrorCovPost, Scalar::all(10));
	Measurement0.create(2, 1);
	Measurement0.setTo(Scalar(0));
	FuturePosition0.create(2, 1);
	FuturePosition0.setTo(Scalar(0));


}

void Vision::GetBallPosition()
{
	unsigned short int i, j, k;
	

	Camera[0] >> Frame[0];
	Camera[1] >> Frame[1];

	for(i=0; i<2; i++)
	{
		cvtColor(Frame[i], FrameHSV[i], CV_BGR2HSV);
		inRange(FrameHSV[i], Scalar(FilterParameters[i][1], FilterParameters[i][3], FilterParameters[i][5]), Scalar(FilterParameters[i][0], FilterParameters[i][2], FilterParameters[i][4]), FrameBIN[i]);
		Position[i][0] = Position[i][1] = 0;
		WhiteCount[i] = 0;
	}

	for(i=0; i<ROWS; i++)
	{
		for(j=0; j<COLS; j++)
		{
			for(k=0; k<2; k++)
			{
				if(FrameBIN[k].data[i*COLS + j] > 200)
				{
					WhiteCount[k]++;
					Position[k][0] += j;
					Position[k][1] += i; 
				}
			}
		}
	}
	
	for(k=0; k<2; k++)
	{
		if(WhiteCount[k] > 20)
		{
			Position[k][0] /= WhiteCount[k];
			Position[k][1] /= WhiteCount[k];
			if(!IsBall1)
			{
				EKF.StatePost.at<float>(0) = atan((float(Position[0][0]) - CENTERX)/(float(Position[0][1]) - CENTERY));
				
				EKF.StatePost.at<float>(1) = 0.0;
				setIdentity(EKF.ErrorCovPost, Scalar::all(10.1));
			}
			IsBall1 = 1;
		}
		else
		{
			Position[k][0] = Position[k][1] = 0;
			IsBall1 = 0;
		}
	}

	#ifdef DEBUG
		circle(Frame[0], Point(Position[0][0], Position[0][1]), 14, Scalar(0, 0, 255), 2, 8, 0);
		circle(Frame[1], Point(Position[1][0], Position[1][1]), 18, Scalar(0, 0, 255), 2, 8, 0);

	#endif

}

void Vision::Filter(char key, bool* startprocess)
{
	if(key == 'r')
	{
		*startprocess = false;
		//cout<<"IF "<<startprocess<<endl;
		EKF.StatePost.at<float>(0) = atan((float(Position[0][0]) - CENTERX)/(float(Position[0][1]) - CENTERY));
		EKF.StatePost.at<float>(1) = -0.1;

		KF.statePost.at<float>(0) = Position[1][0];
		KF.statePost.at<float>(1) = Position[1][1];
	}
	else if(key == 's' || *startprocess)
	{

		Measurement1(0) = Position[1][0];
		Measurement1(1) = Position[1][1];
		KF.predict();
		KF.correct(Measurement1);
		Measurement0(0) = float(Position[0][0]) - CENTERX ;
	        Measurement0(1) = float(Position[0][1]) - CENTERY ;
	        EKF.Predict();
	        EKF.Correct(Measurement0);
		*startprocess = true;
		//cout<<"EIF "<<startprocess<<endl;
	}
	else if(!*startprocess)
	{
		EKF.StatePost.at<float>(0) = atan((float(Position[0][0]) - CENTERX)/(float(Position[0][1]) - CENTERY));
		EKF.StatePost.at<float>(1) = 0;

		KF.statePost.at<float>(0) = Position[1][0];
		KF.statePost.at<float>(1) = Position[1][1];
		//cout<<"E "<<startprocess<<endl;
	}


	#ifdef DEBUG
		circle(Frame[1], Point(KF.statePost.at<float>(0), KF.statePost.at<float>(1)), 5, Scalar(255, 0, 0), 2, 8, 0);
		circle(Frame[0], Point(STRINGLENGTH*sin(EKF.StatePost.at<float>(0)) + CENTERX, STRINGLENGTH*cos(EKF.StatePost.at<float>(0)) + CENTERY), 5, Scalar(255, 0, 0), 2, 8, 0);

	#endif

}

void Vision::Predict()
{
	unsigned short int i;

	Mat temp;
	KF.statePost.copyTo(FuturePosition1);
	EKF.StatePost.copyTo(FuturePosition0);
	for(i=0; i<15; i++)
	{

		temp = KF.transitionMatrix * FuturePosition1;
		temp.copyTo(FuturePosition1);
  		FuturePosition0.at<float>(0) = FuturePosition0.at<float>(0) + FuturePosition0.at<float>(1)*DT;
            	FuturePosition0.at<float>(1) = FuturePosition0.at<float>(1) - ( (G*sin(FuturePosition0.at<float>(0))) + (F*FuturePosition0.at<float>(1)) )*DT;
	}
/*
	for(i=0; i<5; i++)
	{
		temp = KF.transitionMatrix * FuturePosition1;
		temp.copyTo(FuturePosition1);
	}
*/
	#ifdef DEBUG
		circle(Frame[1], Point(FuturePosition1.at<float>(0), FuturePosition1.at<float>(1)), 2, Scalar(255, 255, 255), 2, 8, 0);
		circle(Frame[0], Point(STRINGLENGTH*sin(FuturePosition0.at<float>(0)) + CENTERX, STRINGLENGTH*cos(FuturePosition0.at<float>(0)) + CENTERY), 2, Scalar(255, 255, 255), 2, 8, 0);

	#endif
}

void Vision::Decide(bool* hitready, bool* startprocess)
{
	line(Frame[0], Point(140, 0), Point(140, 480), Scalar(0,0,255), 1, 8);
	if(FuturePosition0.at<float>(0) <= -0.26)
	{
		//cout<<FuturePosition1.at<float>(0)<<endl;
		cout<<STRINGLENGTH*sin(FuturePosition0.at<float>(0)) + CENTERX<<", "<<FuturePosition1.at<float>(0)<<", "<<STRINGLENGTH*cos(FuturePosition0.at<float>(0)) + CENTERY;
		//cout<<" Start Hitting"<<endl;
		*startprocess = false;
		*hitready = true;
	}
	else
	{
		*hitready = false;
	}
}


void Vision::DisplayImages()
{
	imshow("Frame0", Frame[0]);
	imshow("Frame1", Frame[1]);
	//imshow("FrameHSV0", FrameHSV[0]);
	//imshow("FrameHSV1", FrameHSV[1]);
	imshow("FrameBIN0", FrameBIN[0]);
	imshow("FrameBIN1", FrameBIN[1]);
}
#endif
