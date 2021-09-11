add_library( arucoDetection arucoDetection.cpp)

find_package (OpenCV 4.0.0 REQUIRED)
# find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# target_link_libraries( arucoDetection Eigen3::Eigen)
target_link_libraries( arucoDetection ${OpenCV_LIBS} )
target_link_libraries( arucoDetection common )