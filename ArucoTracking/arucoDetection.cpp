// include OpenCV and Aruco libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "arucoDetection.h"
#include "../util/util.h"

#include <iostream>
#include <thread>

#include <queue>
#include <chrono>
#include <functional>

using namespace Eigen;
using namespace rigtorp;

#define TOP_LEFT 0
#define TOP_RIGHT 1
#define BOTTOM_RIGHT 2
#define BOTTOM_LEFT 3

#define PI 3.14159265

// struct holding field data
struct FieldData {
    
    float Max_Input_Value_P;
    float Min_Input_Value_P;
    float Max_Output_Value_P;
    float Min_Output_Value_P;

    float Max_Input_Value_X;
    float Min_Input_Value_X;
    float Max_Output_Value_X;
    float Min_Output_Value_X;

    float Max_Input_Value_Y;
    float Min_Input_Value_Y;
    float Max_Output_Value_Y;
    float Min_Output_Value_Y;

    int Number_of_units;
};

struct CameraSettings {
    
    int VideoPath;

    int VideoWidth;
    int VideoHeight;

    float Contrast;
    float Brightness;
    float Saturation;
    float Gain;
    float Gamma;
    float Sharpness;
};

// queues poses in a single pass
SPSCQueue<AllPoseStates> posesSPSCQueue(2);

// queues images to be processed
SPSCQueue<cv::Mat> imageSPSCQueue(2);

// queues images to be visualized
SPSCQueue<cv::Mat> visualizationSPSCQueue(2);

// holds the relevant field and camera data
FieldData field_data;
CameraSettings camera_settings;

const std::string calibrationFilePath = "../CameraCalibration/calibration1.yml";
const std::string configFilePath = "../config.json";

// get camera calibration parameters from fileName path
void readCameraParameters(cv::Mat *cameraMatrix, cv::Mat *distCoeffs, std::string fileName)
{
    cv::FileStorage fs(fileName , cv::FileStorage::READ);

	fs["camera_matrix"] >> *cameraMatrix;
	fs["distortion_coefficients"] >> *distCoeffs;

	std::cout << "cameraMatrix read = " << *cameraMatrix << std::endl;
	std::cout << "distCoeffs read = " << *distCoeffs << std::endl;
}

void loadFieldData(std::string filePath)
{
    using namespace rapidjson;

    std::ifstream ifs(filePath);
    if ( !ifs.is_open() )
    {
        std::cerr << "Could not open config file for reading!" << std::endl;
        throw FATAL_ERROR;
    }

    IStreamWrapper isw(ifs);
    Document document;
    document.ParseStream(isw);

    assert(document.IsObject());

    if (document.HasMember("FieldData"))
    {
        const Value& field_json = document["FieldData"];

        ////// P
        field_data.Max_Input_Value_P = field_json["Max_Input_Value_P"].GetFloat();
        field_data.Min_Input_Value_P = field_json["Min_Input_Value_P"].GetFloat();
        field_data.Max_Output_Value_P = field_json["Max_Output_Value_P"].GetFloat();
        field_data.Min_Output_Value_P = field_json["Min_Output_Value_P"].GetFloat();

        ////// X
        field_data.Max_Input_Value_X = field_json["Max_Input_Value_X"].GetFloat();
        field_data.Min_Input_Value_X = field_json["Min_Input_Value_X"].GetFloat();
        field_data.Max_Output_Value_X = field_json["Max_Output_Value_X"].GetFloat();
        field_data.Min_Output_Value_X = field_json["Min_Output_Value_X"].GetFloat();

        ////// Y
        field_data.Max_Input_Value_Y = field_json["Max_Input_Value_Y"].GetFloat();
        field_data.Min_Input_Value_Y = field_json["Min_Input_Value_Y"].GetFloat();
        field_data.Max_Output_Value_Y = field_json["Max_Output_Value_Y"].GetFloat();
        field_data.Min_Output_Value_Y = field_json["Min_Output_Value_Y"].GetFloat();

        // number of active units
        field_data.Number_of_units = field_json["Number_of_units"].GetInt();
    }
    else {
        std::cerr << "Config doesnt have FieldData member" << std::endl;
        throw FATAL_ERROR;
    }

    if (document.HasMember("CameraSettings"))
    {
        const Value& camera_json = document["CameraSettings"];

        // camera settings
        camera_settings.VideoPath = camera_json["VideoSource"].GetInt();
        camera_settings.VideoWidth = camera_json["CameraResolutionW"].GetInt();
        camera_settings.VideoHeight = camera_json["CameraResolutionH"].GetInt();
        camera_settings.Contrast = camera_json["Contrast"].GetFloat();
        camera_settings.Brightness = camera_json["Brightness"].GetFloat();
        camera_settings.Saturation = camera_json["Saturation"].GetFloat();
        camera_settings.Gain = camera_json["Gain"].GetFloat();
        camera_settings.Gamma = camera_json["Gamma"].GetFloat();
        camera_settings.Sharpness = camera_json["Sharpness"].GetFloat();
    }
    else {
        std::cerr << "Config doesnt have Camera Settings member" << std::endl;
        throw FATAL_ERROR;
    }

    ifs.close();
}

// add offset to value
int addOffset( int value, int offset)
{
    return abs(value) - offset;
}

// returns a struct holding all pose/id structs per robot, non-blocking function
int getAllPoseStates(AllPoseStates& pose_holder)
{
    if( posesSPSCQueue.front() )
    {
        pose_holder = *posesSPSCQueue.front();
        posesSPSCQueue.pop();

        return 1;
    }
    else return 0;
}

// open video source and start pushing frames to image SPSC queue 
void start_capturing()
{
    
    try {
        loadFieldData(configFilePath);
    }
    catch(int& err){
        std::cerr << "Failed to load field data, aborting.." << std::endl;
        std::abort();
    }
    
    // open video source, set internal buffer to 0
    cv::VideoCapture inputVideo;

    inputVideo.open(camera_settings.VideoPath);
    
    cv::Mat InputImage;

    // try to open video feed
    // if successful push frame to queue
    // if fails, try again untill it succeeds
    while(1) {
        // if the input is opened, read image and push to buffer
        if(inputVideo.isOpened()) {
            
            // Set internal buffer to 0
            inputVideo.set(cv::CAP_PROP_BUFFERSIZE, 1);

            // Set camera parameters
            inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, camera_settings.VideoWidth);
            inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, camera_settings.VideoHeight);

            inputVideo.set(cv::CAP_PROP_CONTRAST, camera_settings.Contrast);
            inputVideo.set(cv::CAP_PROP_BRIGHTNESS, camera_settings.Brightness);
            inputVideo.set(cv::CAP_PROP_SATURATION, camera_settings.Saturation);
            inputVideo.set(cv::CAP_PROP_GAIN, camera_settings.Gain);
            inputVideo.set(cv::CAP_PROP_GAMMA, camera_settings.Gamma);
            inputVideo.set(cv::CAP_PROP_SHARPNESS, camera_settings.Sharpness);

            while(inputVideo.read(InputImage)) {
                
                imageSPSCQueue.try_push(InputImage);
            }
        }
        else {
            std::cerr << "No video source found! " << std::endl;

            while(!inputVideo.open(camera_settings.VideoPath)) {
                std::cout << "Trying to open video source at: " << camera_settings.VideoPath << std::endl;
            }
        }
    }
}

/*
* grab frames in queue, find markers, detect poses and push poses to queue by:
* 1. start frame capturing
* 2. initialize aruco detection infrastructure
* 3. in forever loop:
*      1. grab image, copy for visualization
*      2. detect markers, estimate pose, draw axis on visualizing image
*      3. transform data to x,y coordinates, yaw in {-pi,pi} and ID to per robot pose holders
*      4. try_push poses and visualized imgs to queue */
void start_pose_estimation(const std::string calibrationPath)
{
    std::thread capture_thread(start_capturing);

    // camera calibration parameters
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    // detected IDs with Corners
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    // TODO1 maybe remove reserves
    ids.reserve(field_data.Number_of_units);
    corners.reserve(field_data.Number_of_units);

    // frame holders
    cv::Mat imageCopyVisualize;
    cv::Mat currentImage;

    // temporary pose holder
    ChassisFullState single_pose_holder;
    AllPoseStates pose_holder;

    // camera parameters are read from calibrationFileName
    readCameraParameters(&cameraMatrix, &distCoeffs, calibrationPath);

    // define parameters of the dictionary to be detected
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    Matrix3f new_rotations(3,3);
    float my_yaw;

    // #metrics
    int avgTime = 0;
    #define AVG_NUM_CYCLES 200
    
    while(1) {
        for(int test1 = 0; test1 < AVG_NUM_CYCLES; test1++)
        { //#metrics
        auto start1 = std::chrono::system_clock::now(); //#metrics

        while (!imageSPSCQueue.front());

        currentImage = *imageSPSCQueue.front();
        imageCopyVisualize = currentImage;

        // params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        // params->adaptiveThreshConstant = true;
        
        cv::aruco::detectMarkers(currentImage, dictionary, corners, ids, params);

        cv::aruco::drawDetectedMarkers(imageCopyVisualize, corners, ids);

        // std::cout << "Number of detected markers " << ids.size() << std::endl;

        if (ids.size() > 0) {
            
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            
            // draw axis and corners for each marker
            for(int i=0; i<ids.size(); i++) {

                // find marker center point in pixel space
                float top_left_x = corners.at(i).at(TOP_LEFT).x;
                float top_left_y = corners.at(i).at(TOP_LEFT).y;

                // float top_right_x = corners.at(i).at(TOP_RIGHT).x;
                // float top_right_y = corners.at(i).at(TOP_RIGHT).y;

                // float bottom_left_x = corners.at(i).at(BOTTOM_LEFT).x;
                // float bottom_left_y = corners.at(i).at(BOTTOM_LEFT).y;

                float bottom_right_x = corners.at(i).at(BOTTOM_RIGHT).x;
                float bottom_right_y = corners.at(i).at(BOTTOM_RIGHT).y;

                float half_size_x = (top_left_x - bottom_right_x) / 2;
                float half_size_y = (top_left_y - bottom_right_y) / 2;

                float center_x = top_left_x + half_size_x;
                float center_y = top_left_y + half_size_y;
                
                // find x difference between corner and center
                float center_to_top_left_x = top_left_x - center_x;
                float center_to_top_left_y = top_left_y - center_y;

                // normalize distance to 0-1
                float max_distance = sqrt( (half_size_x*half_size_x) + (half_size_y*half_size_y) );

                float normalized_center_to_top_left_x = MapValueToRange(-max_distance, -1, max_distance, 1, center_to_top_left_x);
                // float normalized_center_to_top_left_y = MapValueToRange(-max_distance, -1, max_distance, 1, center_to_top_left_y);

                // get angle of rotation(yaw)
                float new_yaw = acos(normalized_center_to_top_left_x); //+ (2*PI - 0.785); // rotated by 45 to match the drawn X(red) axis

                // get y distance to determine quadrant
                int y_sign = getSign(center_to_top_left_y);

                // set quadrant sign
                new_yaw = (new_yaw * y_sign);
                float yaw_deg = new_yaw * (180.0/3.14159265);
                std::cout << "The degree value of entered yaw: " << yaw_deg << std::endl;


                // output yaw and position for this marker
                // std::cout << "The new yaw(-PI,PI) is: " << new_yaw << std::endl;
                // std::cout << "The new center(in pixel space) X:" << center_x << std::endl;
                // std::cout << "The new center(in pixel space) Y:" << center_y << std::endl;

                // std::cout << "////////////////////////////////////////" << std::endl;
                
                // msDelay(500);

                // #testing
                cv::aruco::drawAxis(imageCopyVisualize, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

                // transform compact rodrigues representation to rotation matrix
                // cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_64F);
                // cv::Rodrigues(rvecs[i], rot_mat);

                // move from mat to Eigen matrix
                // for(int i = 0; i < 3; i++) {
                //     for(int j = 0; j < 3; j++) {
                //         new_rotations(i,j) = rot_mat.at<double>(i,j);
                //     }
                // }

                // transform rotation matrix to yaw, pitch. roll (in that order)
                // Vector3f YRP_vector = new_rotations.eulerAngles(2, 2, 2);
                // my_yaw = YRP_vector(2,0);
                
                // add pose yaw, x, y and ID
                // map from camera space to coordinate space
                // single_pose_holder.pose_state.q.yaw = my_yaw;
                // single_pose_holder.id = ids[i];
                // pose_holder.poses.push_back(single_pose_holder);

                single_pose_holder.pose_state.q.yaw = new_yaw;
                single_pose_holder.pose_state.q.x = MapValueToRange(field_data.Min_Input_Value_X, field_data.Min_Output_Value_X, field_data.Max_Input_Value_X, field_data.Max_Output_Value_X, tvecs[i][0]);
                single_pose_holder.pose_state.q.y = MapValueToRange(field_data.Min_Input_Value_Y, field_data.Min_Output_Value_Y, field_data.Max_Input_Value_Y, field_data.Max_Output_Value_Y, tvecs[i][1]);
                single_pose_holder.id = ids[i];
                pose_holder.poses.push_back(single_pose_holder);
                
            }
            // push vectored tvecs, rvecs and ids to queue if queue is not full
            posesSPSCQueue.try_push(pose_holder);

            // pop current set of tvecs and rvecs
            pose_holder.poses.clear();
            // msDelay(500);
        }
        else {
            //std::cout << "No markers detected!" << std::endl;
        }

        // #testing
        visualizationSPSCQueue.try_push(imageCopyVisualize);

        // pop frame used for pose estimation
        imageSPSCQueue.pop();

        auto end1 = std::chrono::system_clock::now(); //#metrics
        auto elapsed1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count(); //#metrics
        // std::cout << "The first part took: " << elapsed1  << std::endl; //#metrics
        avgTime += elapsed1; // #metrics


        } // for 100 #metrics
        std::cout << "The avg time per pass is: " << avgTime/AVG_NUM_CYCLES  << std::endl; //#metrics
        std::cout << "The avg fps is: " << 1000/(avgTime/AVG_NUM_CYCLES)  << std::endl; //#metrics
        avgTime = 0;

    }
    capture_thread.join();
}

// run visualization  
void start_visualization()
{
    while(1) {
        
        while(!visualizationSPSCQueue.front());
        cv::imshow("Output", *visualizationSPSCQueue.front());
        char key = (char) cv::waitKey(1);
        if (key == 'q')
            break;

        visualizationSPSCQueue.pop();
    }
}

// start detecting markers and placing found in SPSC queue
void start_aruco_detection()
{
    std::thread estimation_thread(start_pose_estimation, calibrationFilePath);
    std::thread visualization_thread(start_visualization);
    
    //std::cout << "we got past the thread joins" << std::endl;

    estimation_thread.join();
    visualization_thread.join();
}
