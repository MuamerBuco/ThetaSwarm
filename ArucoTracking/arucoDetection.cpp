
// include OpenCV and Aruco libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "arucoDetection.h"
#include "../CommonFunctions/common.h"
#include "../3rdParty/SPSCQueue/include/rigtorp/SPSCQueue.h"

#include <iostream>
#include <thread>

#include <queue>
#include <chrono>
#include <functional>
#include <X11/Xlib.h>

using namespace Eigen;
using namespace rigtorp;

#define MAX_NUMBER_OF_ACTIVE_UNITS 6

SPSCQueue<cv::Mat> imageSPSCQueue(2);
SPSCQueue<cv::Mat> visualizationSPSCQueue(2);

SPSCQueue<AllPosesInPass> posesSPSCQueue(2);

// const std::string videoIP = "rtsp://192.168.1.79:4847/h264_ulaw.sdp";
//const std::string videoPath = "rtsp://192.168.1.97:4847/h264_ulaw.sdp";
// const std::string videoIP = "rtsp://10.237.139.214:4847/h264_ulaw.sdp";

int videoPath = 2;

const std::string calibrationFilePath = "/home/eon/Thesis/ThesisEmbedded/ESP32/TestFirmware/ThesisRobot/TestUDPClient/CameraCalibration/calibration1.yml";

void readCameraParameters(cv::Mat *cameraMatrix, cv::Mat *distCoeffs, std::string fileName)
{
    cv::FileStorage fs(fileName , cv::FileStorage::READ);

	fs["camera_matrix"] >> *cameraMatrix;
	fs["distortion_coefficients"] >> *distCoeffs;

	std::cout << "cameraMatrix read = " << *cameraMatrix << std::endl;
	std::cout << "distCoeffs read = " << *distCoeffs << std::endl;
}

int addOffset( int value, int offset)
{
    value = abs(value) - offset;

    return value;
}

void start_capturing()
{
    // open video source, set internal buffer to 0
    cv::VideoCapture inputVideo;
    inputVideo.set(cv::CAP_PROP_BUFFERSIZE, 0);
    inputVideo.open(videoPath);
    
    cv::Mat InputImage;

    while(1) {
        // if the input is opened, read image and push to buffer
        if(inputVideo.isOpened()) {
            while(inputVideo.read(InputImage)) {
                
                imageSPSCQueue.try_push(InputImage);
            }
        }
        else {
            std::cout << "No video source found! " << std::endl;

            while(!inputVideo.open(videoPath)) {
                //videoPath++;
                std::cout << "Trying to open video source at: " << videoPath << std::endl;
            }
        }
    }
}

void start_pose_estimation(const std::string calibrationPath)
{
    std::thread capture_thread(start_capturing);

    // camera calibration parameters
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    // detected IDs with Corners
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    ids.reserve(MAX_NUMBER_OF_ACTIVE_UNITS);
    corners.reserve(MAX_NUMBER_OF_ACTIVE_UNITS);

    // frame holders
    cv::Mat imageCopyVisualize;
    cv::Mat currentImage;

    // temporary pose holder
    ChassisPoseAndID single_pose_holder;
    AllPosesInPass pose_holder;

    // camera parameters are read from calibrationFileName
    readCameraParameters(&cameraMatrix, &distCoeffs, calibrationPath);

    // define parameters of the dictionary to be detected
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    Matrix3f new_rotations(3,3);
    int my_yaw;

    while(1) {
        for(int test1 = 0; test1 < 200; test1++){
        auto start1 = std::chrono::system_clock::now();

        while (!imageSPSCQueue.front());
        // #testing
        currentImage = *imageSPSCQueue.front();
        //currentImage = currentImage(cv::Rect(500,500,100,100));
        imageCopyVisualize = currentImage;
        
        cv::aruco::detectMarkers(currentImage, dictionary, corners, ids);

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

                // transform rotation matrix to yaw, pitch. roll (in that order)
                Vector3f YRP_vector = new_rotations.eulerAngles(2, 2, 2);
                //std::cout << "The yaw before mapping: " << YRP_vector(0,0) << std::endl;
                //my_yaw = static_cast<int>( MapValueToRange(0, -180, 3.14, 180, YRP_vector(0,0)) );
                //my_yaw = static_cast<int>( MapValueToRange(0, 0, 3.14, 360, YRP_vector(1,0)) );
                //std::cout << "The yaw raw value: " << YRP_vector(2,0) << std::endl;
                //my_yaw = addOffset(my_yaw, 0);
                //std::cout << "The yaw after mapping: " << my_yaw << std::endl;

                my_yaw = static_cast<int>(YRP_vector(2,0) * 100);
                //std::cout << "The yaw saved value: " << my_yaw << std::endl;
                

                //std::cout << "The tvec 0: " << tvecs[i][0] << std::endl;
                //std::cout << "The tvec 1: " << tvecs[i][1] << std::endl;
                // add pose yaw, x, y and ID
                single_pose_holder.yaw = my_yaw;
                single_pose_holder.x = static_cast<int>( MapValueToRange(MAX_INPUT_VALUE_X, MAX_OUPUT_VALUE_X, MIN_INPUT_VALUE_X, MIN_OUPUT_VALUE_X, tvecs[i][0]) );
                single_pose_holder.y = static_cast<int>( MapValueToRange(MAX_INPUT_VALUE_Y, MAX_OUPUT_VALUE_Y, MIN_INPUT_VALUE_Y, MIN_OUPUT_VALUE_Y, tvecs[i][1]) );
                single_pose_holder.id = ids[i];
                pose_holder.poses.push_back(single_pose_holder);
                
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

        // #testing for drawn axis stability, only push visualized frames that contain markers if queue isnt full
        visualizationSPSCQueue.try_push(imageCopyVisualize);

        // pop frame used for pose estimation
        imageSPSCQueue.pop();

        auto end1 = std::chrono::system_clock::now();
        auto elapsed1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count();
        std::cout << "The first part took: " << elapsed1  << std::endl;

        }
    }
    capture_thread.join();
}

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

void start_aruco_detection()
{
    std::thread estimation_thread(start_pose_estimation, calibrationFilePath);
    std::thread visualization_thread(start_visualization);
    
    //std::cout << "we got past the thread joins" << std::endl;

    estimation_thread.join();
    visualization_thread.join();
}

// returns a struct holding all pose/id structs per robot, blocking
AllPosesInPass getAllPosesAndIDs()
{
    while(!posesSPSCQueue.front());
    AllPosesInPass pose_holder = *posesSPSCQueue.front();

    posesSPSCQueue.pop();

    return pose_holder;
}

void whatever1()
{
    auto start1 = std::chrono::system_clock::now();
    
    auto end1 = std::chrono::system_clock::now();
    auto elapsed1 = end1 - start1;
    //std::cout << "The action took: " << elapsed1.count() << '\n';
}