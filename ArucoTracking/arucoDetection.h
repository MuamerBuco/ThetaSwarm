#ifndef ARUCO_TRACKING_H
#define ARUCO_TRACKING_H

// include 3rd party libs
#include "../3rdParty/SPSCQueue/include/rigtorp/SPSCQueue.h"
#include "../3rdParty/rapidjson/filereadstream.h"
#include "../3rdParty/rapidjson/rapidjson.h"
#include "../3rdParty/rapidjson/document.h"
#include "../3rdParty/rapidjson/istreamwrapper.h"

#include "../util/util.h"

int getAllPoseStates(AllPoseStates& pose_holder);
FieldData getFieldData();
void start_aruco_detection();

#endif //ARUCO_TRACKING_H