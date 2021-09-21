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
    int VideoPath;
};

// queues poses in a single pass
SPSCQueue<AllPoseStates> posesSPSCQueue(2);

// queues images to be processed
SPSCQueue<cv::Mat> imageSPSCQueue(2);

// queues images to be visualized
SPSCQueue<cv::Mat> visualizationSPSCQueue(2);

// holds the relevant field data
FieldData field_data;

int videoPath = 2;

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
        assert(field_json["Max_Input_Value_P"].IsNumber());
        field_data.Max_Input_Value_P = field_json["Max_Input_Value_P"].GetFloat();

        assert(field_json["Min_Input_Value_P"].IsNumber());
        field_data.Min_Input_Value_P = field_json["Min_Input_Value_P"].GetFloat();

        assert(field_json["Max_Output_Value_P"].IsNumber());
        field_data.Max_Output_Value_P = field_json["Max_Output_Value_P"].GetFloat();

        assert(field_json["Min_Output_Value_P"].IsNumber());
        field_data.Min_Output_Value_P = field_json["Min_Output_Value_P"].GetFloat();

        ////// X
        assert(field_json["Max_Input_Value_X"].IsNumber());
        field_data.Max_Input_Value_X = field_json["Max_Input_Value_X"].GetFloat();

        assert(field_json["Min_Input_Value_X"].IsNumber());
        field_data.Min_Input_Value_X = field_json["Min_Input_Value_X"].GetFloat();

        assert(field_json["Max_Output_Value_X"].IsNumber());
        field_data.Max_Output_Value_X = field_json["Max_Output_Value_X"].GetFloat();

        assert(field_json["Min_Output_Value_X"].IsNumber());
        field_data.Min_Output_Value_X = field_json["Min_Output_Value_X"].GetFloat();

        ////// Y
        assert(field_json["Max_Input_Value_Y"].IsNumber());
        field_data.Max_Input_Value_Y = field_json["Max_Input_Value_Y"].GetFloat();

        assert(field_json["Min_Input_Value_Y"].IsNumber());
        field_data.Min_Input_Value_Y = field_json["Min_Input_Value_Y"].GetFloat();

        assert(field_json["Max_Output_Value_Y"].IsNumber());
        field_data.Max_Output_Value_Y = field_json["Max_Output_Value_Y"].GetFloat();

        assert(field_json["Min_Output_Value_Y"].IsNumber());
        field_data.Min_Output_Value_Y = field_json["Min_Output_Value_Y"].GetFloat();

        // number of active units
        assert(field_json["Number_of_units"].IsNumber());
        field_data.Number_of_units = field_json["Number_of_units"].GetInt();

        assert(field_json["VideoSource"].IsNumber());
        field_data.VideoPath = field_json["VideoSource"].GetInt();
    }
    else {
        std::cerr << "Config doesnt have FieldData member" << std::endl;
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
    inputVideo.set(cv::CAP_PROP_BUFFERSIZE, 0);
    inputVideo.open(field_data.VideoPath);
    
    cv::Mat InputImage;

    // try to open video feed
    // if successful push frame to queue
    // if fails, try again untill it succeeds
    while(1) {
        // if the input is opened, read image and push to buffer
        if(inputVideo.isOpened()) {
            while(inputVideo.read(InputImage)) {
                
                imageSPSCQueue.try_push(InputImage);
            }
        }
        else {
            std::cerr << "No video source found! " << std::endl;

            while(!inputVideo.open(field_data.VideoPath)) {
                std::cout << "Trying to open video source at: " << videoPath << std::endl;
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

    // camera parameters are read from calibrationFileName
    readCameraParameters(&cameraMatrix, &distCoeffs, calibrationPath);

    // define parameters of the dictionary to be detected
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    Matrix3f new_rotations(3,3);
    float my_yaw;
    
    while(1) {
        for(int test1 = 0; test1 < 200; test1++)
        { //#metrics
        auto start1 = std::chrono::system_clock::now(); //#metrics

        while (!imageSPSCQueue.front());

        // #testing
        currentImage = *imageSPSCQueue.front();
        //currentImage = currentImage(cv::Rect(500,500,100,100));
        imageCopyVisualize = currentImage;
        
        cv::aruco::detectMarkers(currentImage, dictionary, corners, ids);

        // std::cout << "Number of detected markers " << ids.size() << std::endl;

        // msDelay(1); //#delay


        if (ids.size() > 0) {
            
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

            cv::aruco::drawDetectedMarkers(imageCopyVisualize, corners, ids);
            
            // draw axis and corners for each marker
            for(int i=0; i<ids.size(); i++) {
                
                // #testing
                cv::aruco::drawAxis(imageCopyVisualize, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

                // transform compact rodrigues representation to rotation matrix
                cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_64F);
                cv::Rodrigues(rvecs[i], rot_mat);

                // move from mat to Eigen matrix
                for(int i = 0; i < 3; i++) {
                    for(int j = 0; j < 3; j++) {
                        new_rotations(i,j) = rot_mat.at<double>(i,j);
                    }
                }

                // TODO1 clean up this shit

                // transform rotation matrix to yaw, pitch. roll (in that order)
                Vector3f YRP_vector = new_rotations.eulerAngles(2, 2, 2);
                //std::cout << "The yaw before mapping: " << YRP_vector(0,0) << std::endl;
                //my_yaw = static_cast<int>( MapValueToRange(0, -180, 3.14, 180, YRP_vector(0,0)) );
                //my_yaw = static_cast<int>( MapValueToRange(0, 0, 3.14, 360, YRP_vector(1,0)) );

                //std::cout << "The yaw raw value: " << YRP_vector(2,0) << std::endl;
                //my_yaw = addOffset(my_yaw, 0);
                //std::cout << "The yaw after mapping: " << my_yaw << std::endl;

                // TODO1 check if this is suppose to be degree or rad, tho probably -pi, pi
                my_yaw = YRP_vector(2,0);
                // std::cout << "The yaw saved value: " << my_yaw << std::endl;
                

                // std::cout << "The tvec 0: " << tvecs[i][0] << std::endl;
                // std::cout << "The tvec 1: " << tvecs[i][1] << std::endl;
                
                // add pose yaw, x, y and ID
                // map from camera space to coordinate space
                single_pose_holder.pose_state.q.yaw = my_yaw;
                single_pose_holder.pose_state.q.x = MapValueToRange(field_data.Min_Input_Value_X, field_data.Min_Output_Value_X, field_data.Max_Input_Value_X, field_data.Max_Output_Value_X, tvecs[i][0]);
                single_pose_holder.pose_state.q.y = MapValueToRange(field_data.Min_Input_Value_Y, field_data.Min_Output_Value_Y, field_data.Max_Input_Value_Y, field_data.Max_Output_Value_Y, tvecs[i][1]);
                single_pose_holder.id = ids[i];
                pose_holder.poses.push_back(single_pose_holder);



                // std::cout << "In aruco: the newly found value for y: " << single_pose_holder.pose_state.q.y << std::endl;
                // std::cout << "In aruco: field data Max_Output_Value_Y: " << field_data.Max_Output_Value_Y << std::endl;
                // std::cout << "In aruco: the pushed id: " << single_pose_holder.id << std::endl;
                
                //std::cout << "rvecs: " << std::endl;
                // for (auto vec : rvecs)
                //    std::cout << vec << std::endl;
         
                //std::cout << "tvecs: " << std::endl; 
                // for (auto vec : tvecs)
                //     std::cout << vec << std::endl;
            }
            // push vectored tvecs, rvecs and ids to queue if queue is not full
            posesSPSCQueue.try_push(pose_holder);

            // #testing for drawn axis stability, only push visualized frames that contain markers if queue isnt full
            visualizationSPSCQueue.try_push(imageCopyVisualize);

            // pop current set of tvecs and rvecs
            pose_holder.poses.clear();
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

        } // for 100 #metrics
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
