#include "navigation.h"
#include "../../../../3rdParty/SPSCQueue/include/rigtorp/SPSCQueue.h"

#ifndef STRING
#define STRING
#include <string>
#endif

#include <iostream>
#include <vector>

using namespace rigtorp;
using namespace Eigen;

SPSCQueue<AllTargetPoses> target_poses_buffer(10);

void fill_target_pose_buffer()
{
    for(int i = 0; i < 10; i++) {
        
        TargetPose target_pose1;

        target_pose1.p = 0;
        target_pose1.x = 160;
        target_pose1.y = 100;
        target_pose1.id = 15; //to be set for testing

        AllTargetPoses allTargetPoses;

        allTargetPoses.allPosesVector.push_back(target_pose1);

        //std::cout << "the fill target poses single target" << target_pose1.x << std::endl;

        target_poses_buffer.try_push(allTargetPoses);
    }
}

// returns a vector holding all pose/id targets per robot
AllTargetPoses getAllTargetPosesAndIDs()
{
    if(target_poses_buffer.front()) {
        
        AllTargetPoses target_holder = *target_poses_buffer.front();

        //std::cout << "The get all target poses x" << target_holder.allPosesVector[0].x << std::endl;

        target_poses_buffer.pop();
        return target_holder;
    }
    else {
        throw std::invalid_argument( "target buffer empty! \n" );
    }    
}

