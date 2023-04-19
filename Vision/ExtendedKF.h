#ifndef ExtendedKF_H_
#define ExtendedKF_H_

#define DT 0.033
#define STRINGLENGTH 680
#define CENTERX 330
#define CENTERY -310
#define G 6.3 //6.5
#define F 0.07 //0.075

class ExtendedKalmanFilter
{
    public:
        Mat StatePre;           
        Mat StatePost;            
        Mat ControlMatrix;      
        Mat TransitionJacobian;
        Mat MeasurementJacobian; 
        Mat Measurement;  
        Mat ProcessNoiseCov;    
        Mat MeasurementNoiseCov;
        Mat ErrorCovPre;        
        Mat Gain;               
        Mat ErrorCovPost;   
 
        Mat temp1;
        Mat temp2;
        Mat temp3;
        Mat temp4;
        Mat temp5;
         
        ExtendedKalmanFilter();
        ExtendedKalmanFilter(int, int, int , int);
        const Mat& predict(const Mat& control=Mat());
        const Mat& correct(const Mat& measurement);
        void Initialization(int, int, int, int);
        void GetTransitionJacobian();
        void GetMeasurementJacobian();
        void ProcessModel();
        void MeasurementModel();
        void Predict();
        void Correct(const Mat&);
};

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
}

ExtendedKalmanFilter::ExtendedKalmanFilter(int dynamParams, int measureParams, int controlParams, int type)
{
    Initialization(dynamParams, measureParams, controlParams, type);
}
 
void ExtendedKalmanFilter::Initialization(int DP, int MP, int CP = 0, int type = CV_32F)
{
    CV_Assert( DP > 0 && MP > 0 );
    CP = std::max(CP, 0);
 
    StatePre = Mat::zeros(DP, 1, type);
        StatePost = Mat::zeros(DP, 1, type);
        TransitionJacobian = Mat::eye(DP, DP, type);
    MeasurementJacobian = Mat::eye(MP, DP, type);
 
    ProcessNoiseCov = Mat::eye(DP, DP, type);
    Measurement = Mat::zeros(MP, 1, type);
    MeasurementNoiseCov = Mat::eye(MP, MP, type);
 
    ErrorCovPre = Mat::zeros(DP, DP, type);
    ErrorCovPost = Mat::zeros(DP, DP, type);
    Gain = Mat::zeros(DP, MP, type);
 
    if( CP > 0 )
        ControlMatrix = Mat::zeros(DP, CP, type);
    else
        ControlMatrix.release();
 
    temp1.create(DP, DP, type);
    temp2.create(MP, DP, type);
    temp3.create(MP, MP, type);
    temp4.create(MP, DP, type);
    temp5.create(MP, 1, type);
}
 
void ExtendedKalmanFilter::GetTransitionJacobian()
{
    TransitionJacobian.at<float>(0, 0) = 1.0;
    TransitionJacobian.at<float>(0, 1) = DT;
    TransitionJacobian.at<float>(1, 0) = -G*DT*cos(StatePost.at<float>(0));
    TransitionJacobian.at<float>(1, 1) = 1.0 - F*DT;
}
 
void ExtendedKalmanFilter::GetMeasurementJacobian()
{
    MeasurementJacobian.at<float>(0, 0) = STRINGLENGTH*cos(StatePre.at<float>(0));
    MeasurementJacobian.at<float>(0, 1) = 0.0;
    MeasurementJacobian.at<float>(1, 0) = -STRINGLENGTH*sin(StatePre.at<float>(0));
    MeasurementJacobian.at<float>(1, 1) = 0.0;
}
 
void ExtendedKalmanFilter::ProcessModel()
{
 
    StatePre.at<float>(0) = StatePost.at<float>(0) + StatePost.at<float>(1)*DT;
    StatePre.at<float>(1) = StatePost.at<float>(1) - G*sin(StatePost.at<float>(0))*DT - F*StatePost.at<float>(1)*DT;
 
}
 
void ExtendedKalmanFilter::MeasurementModel()
{
    Measurement.at<float>(0) = STRINGLENGTH*sin(StatePre.at<float>(0));
    Measurement.at<float>(1) = STRINGLENGTH*cos(StatePre.at<float>(0));
}
 
void ExtendedKalmanFilter::Predict()
{
    ProcessModel();
    GetTransitionJacobian();
    temp1 = TransitionJacobian*ErrorCovPost;
    gemm(temp1, TransitionJacobian, 1, ProcessNoiseCov, 1, ErrorCovPre, GEMM_2_T);
}
 
void ExtendedKalmanFilter::Correct(const Mat& measurement)
{
    GetMeasurementJacobian();
    temp2 = MeasurementJacobian*ErrorCovPre;
    gemm(temp2, MeasurementJacobian, 1, MeasurementNoiseCov, 1, temp3, GEMM_2_T);
    solve(temp3, temp2, temp4, DECOMP_SVD);
    Gain = temp4.t();
    MeasurementModel();
    temp5 = measurement - Measurement;
    StatePost = StatePre + Gain*temp5;
    ErrorCovPost = ErrorCovPre - Gain*temp2;    
}

#endif
