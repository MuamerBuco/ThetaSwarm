// include OpenCV and Aruco libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "arucoDetection.h"
// #include "../Metrics/metrics.h"

#include <Eigen/Dense>

#include <vector>
#include <fstream>

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

// holds the relevant field, camera and aruco data
FieldData field_data;
CameraSettings camera_settings;
ArucoParams aruco_parameters;

// queues poses in a single pass
SPSCQueue<AllPoseStates> posesSPSCQueue(2);

// queues images to be processed
SPSCQueue<cv::Mat> imageSPSCQueue(2);

// queues images to be visualized
SPSCQueue<cv::Mat> visualizationSPSCQueue(2);

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

        ////// YAW
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

        field_data.dataLoaded = true;
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
        std::cerr << "Config doesnt have CameraSettings member" << std::endl;
        throw FATAL_ERROR;
    }

    if (document.HasMember("ArucoParameters"))
    {
        const Value& camera_json = document["ArucoParameters"];

        // aruco settings
        aruco_parameters.minMarkerPerimeterRate = camera_json["minMarkerPerimeterRate"].GetFloat();
        aruco_parameters.maxMarkerPerimeterRate = camera_json["maxMarkerPerimeterRate"].GetFloat();
        aruco_parameters.adaptiveThreshWinSizeMin = camera_json["adaptiveThreshWinSizeMin"].GetFloat();
        aruco_parameters.adaptiveThreshWinSizeMax = camera_json["adaptiveThreshWinSizeMax"].GetFloat();
        aruco_parameters.adaptiveThreshWinSizeStep = camera_json["adaptiveThreshWinSizeStep"].GetFloat();
    }
    else {
        std::cerr << "Config doesnt have ArucoParameters member" << std::endl;
        throw FATAL_ERROR;
    }

    ifs.close();
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

// return the field data if the data was loaded properly, throw FATAL_ERROR if not
FieldData getFieldData()
{
    if(field_data.dataLoaded) return field_data;
    else throw FATAL_ERROR;
}

// open video source and start pushing frames to image SPSC queue 
void start_capturing()
{
    
    try {
        loadFieldData(configFilePath);
    }
    catch(int& err){
        std::cerr << "Failed to load field data from config, aborting.." << std::endl;
        std::exit( EXIT_FAILURE );
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
            inputVideo.set(cv::CAP_PROP_FPS, 120);

            while(inputVideo.read(InputImage))
            {
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

    ids.reserve(field_data.Number_of_units);
    corners.reserve(field_data.Number_of_units);

    // frame holders
    cv::Mat imageCopyVisualize;
    cv::Mat currentImage;

    // temporary pose holder
    ChassisFullState single_pose_holder;
    AllPoseStates pose_holder;

    struct cornerPosition {
        float x;
        float y;
    };

#ifdef SHOW_METRICS
    // performance metric variables
    typedef std::chrono::milliseconds ms;
    std::chrono::duration<double> time_for_latest_pass;
    float framerate_counter = 0.1;
    float framesuccess_counter = 0.1;
    float totalElapsed = 0;
#endif

    // camera parameters are read from calibrationFileName
    readCameraParameters(&cameraMatrix, &distCoeffs, calibrationPath);

    // define parameters of the dictionary to be detected
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    Matrix3f new_rotations(3,3);
    float my_yaw;
    cornerPosition top_left;
    cornerPosition top_right;
    
    while(1) {
        while(totalElapsed < 1000)
        {
            // Use aruco parameters set in config file
            params->minMarkerPerimeterRate = aruco_parameters.minMarkerPerimeterRate;
            params->maxMarkerPerimeterRate = aruco_parameters.maxMarkerPerimeterRate;

            params->adaptiveThreshWinSizeMin = aruco_parameters.adaptiveThreshWinSizeMin;
            params->adaptiveThreshWinSizeMax = aruco_parameters.adaptiveThreshWinSizeMax;
            params->adaptiveThreshWinSizeStep = aruco_parameters.adaptiveThreshWinSizeStep;

            // start frame timer
            auto timer_start = std::chrono::system_clock::now();

            // wait for camera to push frame, store it in variable
            while (!imageSPSCQueue.front());
            currentImage = *imageSPSCQueue.front();
            
            cv::aruco::detectMarkers(currentImage, dictionary, corners, ids, params);
            cv::aruco::drawDetectedMarkers(currentImage, corners, ids);

            // std::cout << "Number of detected markers " << ids.size() << std::endl;

            if (ids.size() > 0) {
                
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, 0.07, cameraMatrix, distCoeffs, rvecs, tvecs);
                
                // draw axis and corners for each marker
                for(int i=0; i<ids.size(); i++) {

                    top_left.x = corners.at(i).at(TOP_LEFT).x;
                    top_left.y = corners.at(i).at(TOP_LEFT).y;

                    top_right.x = corners.at(i).at(TOP_RIGHT).x;
                    top_right.y = corners.at(i).at(TOP_RIGHT).y;

                    cornerPosition difference;
                    difference.x = top_right.x - top_left.x;
                    difference.y = top_right.y - top_left.y;

                    float max_length = sqrt( (difference.x*difference.x) + (difference.y*difference.y) );

                    float normalized = MapValueToRange(-max_length, -1, max_length, 1, difference.x);

                    float new_yaw = acos(normalized);

                    // get y distance to determine quadrant
                    int y_sign = getSign(difference.y);

                    new_yaw = (new_yaw * y_sign);

                    std::cout << std::endl << "The yaw: " << new_yaw << std::endl;

                    ///////////////////////////////////////////////////////// OLD YAW
                    // // find marker center point in pixel space
                    // float top_left_x = corners.at(i).at(TOP_LEFT).x;
                    // float top_left_y = corners.at(i).at(TOP_LEFT).y;

                    // // float top_right_x = corners.at(i).at(TOP_RIGHT).x;
                    // // float top_right_y = corners.at(i).at(TOP_RIGHT).y;

                    // // float bottom_left_x = corners.at(i).at(BOTTOM_LEFT).x;
                    // // float bottom_left_y = corners.at(i).at(BOTTOM_LEFT).y;

                    // float bottom_right_x = corners.at(i).at(BOTTOM_RIGHT).x;
                    // float bottom_right_y = corners.at(i).at(BOTTOM_RIGHT).y;

                    // float half_size_x = (top_left_x - bottom_right_x) / 2;
                    // float half_size_y = (top_left_y - bottom_right_y) / 2;

                    // float center_x = top_left_x + half_size_x;
                    // float center_y = top_left_y + half_size_y;
                    
                    // // find x difference between corner and center
                    // float center_to_top_left_x = top_left_x - center_x;
                    // float center_to_top_left_y = top_left_y - center_y;

                    // // normalize distance to 0-1
                    // float max_distance = sqrt( (half_size_x*half_size_x) + (half_size_y*half_size_y) );

                    // float normalized_center_to_top_left_x = MapValueToRange(-max_distance, -1, max_distance, 1, center_to_top_left_x);
                    // // float normalized_center_to_top_left_y = MapValueToRange(-max_distance, -1, max_distance, 1, center_to_top_left_y);

                    // // get angle of rotation(yaw)
                    // float new_yaw = acos(normalized_center_to_top_left_x);

                    // // get y distance to determine quadrant
                    // int y_sign = getSign(center_to_top_left_y);

                    // // set quadrant sign
                    // new_yaw = (new_yaw * y_sign) - 0.78539; // rotated by 45 to match the drawn X(red) axis
                    // // float yaw_deg = new_yaw * (180.0/3.14159265);
                    /////////////////////////////////////////////////////////////////////////////////////////////////////////////


                    // std::cout << "The degree value of entered yaw: " << yaw_deg << std::endl;

                    // #testing
                    // visualizationSPSCQueue.try_push(currentImage);

                    // std::cout << "The X value in camera space: " << tvecs[i][0] << std::endl;
                    // std::cout << "The Y value in camera space: " << tvecs[i][1] << std::endl;

                    // msDelay(500);
                    cv::aruco::drawAxis(currentImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

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

                framesuccess_counter++;
            }
            else {
                //std::cout << "No markers detected!" << std::endl;
            }

            // #testing
            visualizationSPSCQueue.try_push(currentImage);

            // pop frame used for pose estimation
            imageSPSCQueue.pop();

#ifdef SHOW_METRICS
            framerate_counter++;

            auto timer_end = std::chrono::system_clock::now();
            time_for_latest_pass = timer_end - timer_start;
            ms milis = std::chrono::duration_cast<ms>(time_for_latest_pass);
            totalElapsed += milis.count();
#endif 
            
        }
        
#ifdef SHOW_METRICS
        std::clog << "Framerate(FPS): " << static_cast<int>(framerate_counter) << std::endl;
        std::clog << "Successful frames per second: " << static_cast<int>(framesuccess_counter) << std::endl;
        std::clog << "Percentage detection success(%): " << static_cast<int>(framesuccess_counter/framerate_counter * 100) << std::endl;
        std::clog << std::endl;

        totalElapsed = 0;
        framerate_counter = 0.1;
        framesuccess_counter = 0.1;
#endif
    }

    // aruco_metrics.join();
    capture_thread.join();
}

// run visualization  
void start_visualization()
{
    while(1) 
    {
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
    
    estimation_thread.join();
    visualization_thread.join();
}
