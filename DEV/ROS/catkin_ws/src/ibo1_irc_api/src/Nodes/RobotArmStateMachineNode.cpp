#include <ros/ros.h>


// Protocol includes
#include <ibo1_irc_api/Protocol.h>
#include <ibo1_irc_api/ProtocolAPI/InternalProtocolDefinition.h>

//Move it includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// Transform listeners
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// Transform publishers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// geometry_msg imports
#include <geometry_msgs/Pose.h>

// Self defined messages include
#include <ibo1_irc_api/ChessCells.h>

// Including own headers
#include <ibo1_irc_api/Utility/DataChecker.h>
#include <ibo1_irc_api/Utility/DataCreator.h>

// C++ includes
#include <math.h>


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //

// For ros subscribers & publisher
ibo1_irc_api::Protocol returnedProtocol;
ibo1_irc_api::ChessCells returnedChessCells;

ros::Publisher* systemStateMachine_pub_ptr;


// PlanningInterface variables
moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_ptr;
std::vector<std::string> object_ids = {"my_front_ground", "my_back_ground", "my_right_ground", "my_left_ground", "my_chess_board", "my_external_camera"};
static const float chessBoardPosition[3] = {0, 0.3625, 0.018};
static const float chessBoardDimension[3] = {0.46, 0.46, 0.04};
static const float externalCameraPosition[3] = {0, 0.3625, 1}; 
static const float externalCameraDimension[3] = {0.05, 0.05, 0.025};


//Robot Arm State Machine Variables
int robotArmState = 0;
int pickDropState = 0;
BYTE initialSender = (BYTE)0x00;
int cellPosPickup = -1;
int cellPosDrop = -1;
geometry_msgs::TransformStamped startFrameTransform;
geometry_msgs::TransformStamped endFrameTransform;



// Transform global variables
tf2_ros::Buffer* tfBuffer_ptr;

string goalFrames[] = {"target_drop_frame", "target_pickup_frame", "graveyard_frame", "restposition_frame"};

static const tf2::Quaternion quaternionPickDrop(0.000, 1.000, 0.000, 0.000);
static const tf2::Quaternion quaternionRestPosition(0.716, 0.698, 0.000, 0.000);


// Height and transform offsets
double chessPieceRemoveIntermediateStandardOffsetHeigh = 0.25;
double chessPieceRemovePickUpStandardOffsetHeigh = 0.015;
double chessPieceRemoveGraveYardIntermediateStandardOffsetHeigh = 0.25;
double chessPieceRemoveGraveYardDropStandardOffsetHeigh = 0.1;


double chessPieceMoveIntermediateStandardOffsetHeight = 0.25;
double chessPieceMovePickUpStandardOffsetHeight = 0.015;
double chessPieceMoveDropIntermediateStandardOffsetHeight = 0.25;
double chessPieceMoveDropStandardOffsetHeigh = 0.095;

// Removing a piece offsets
double removePieceOffsets[4][3] = {
    {0, -0.02, chessPieceRemoveIntermediateStandardOffsetHeigh},
    {0, -0.0125, chessPieceRemovePickUpStandardOffsetHeigh},
    {-0.2, 0, chessPieceRemoveGraveYardIntermediateStandardOffsetHeigh},
    {0, 0, chessPieceRemoveGraveYardDropStandardOffsetHeigh}
};

// Picking up a piece offsets
double movePieceOffsets[4][3] = {
    {0, -0.02, chessPieceMoveIntermediateStandardOffsetHeight},
    {0, -0.0125, chessPieceMovePickUpStandardOffsetHeight},
    {0, -0.02, chessPieceMoveDropIntermediateStandardOffsetHeight},
    {0, -0.0125, chessPieceMoveDropStandardOffsetHeigh},
};

// Speed values

// Robot variables
moveit::planning_interface::MoveGroupInterface* move_group_arm_ptr;
moveit::planning_interface::MoveGroupInterface* move_group_gripper_ptr;

static const double gripperValues[2] = {0.5, 0.6};  

// Speed values arm
static const double speedValuesArm[2][2] = {
    {1.0, 1.0},
    {0.5, 0.5}
};

bool gripperFinished = false;
bool transformFramesUpdated = false;


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////// //
    // Callbacks. //
    // ////////// //
void robotArmStateMachineMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    cout << "----------------------------------------------------------" << endl;
    cout << "I received following on RobotArmStateMachine: " << endl;
    cout << "I received from: " << (int)returnedProtocol.sender << endl;
    cout << "CmdByte: " << (int)returnedProtocol.cmd << endl;
    cout << "----------------------------------------------------------" << endl;
}

void chessCellDetectionMessageReceived(const ibo1_irc_api::ChessCells& msg){
    returnedChessCells = msg;
    // cout << "----------------------------------------------------------" << endl;
    // cout << "I received following on createTarget from chessCellDetection: " << endl;
    // cout << "----------------------------------------------------------" << endl;
}

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ///////////////// //
    // Setup Functions.  //
    // ///////////////// //
    void settingUpPlanningInterface(){

        //Clearing previous planning scene objects
        planning_scene_interface_ptr->removeCollisionObjects(object_ids);


        ROS_INFO("Adding ground to planning scene");

        // Adding/Removing Objects and Attaching/Detaching Objects
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        //
        // Define a collision object ROS message.
        //moveit_msgs::CollisionObject collision_object;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(6);


        // Ground infront
        //____________________________________________________________________________
        // The id of the object is used to identify it.
        collision_objects[0].id = "my_front_ground";
        collision_objects[0].header.frame_id = "world";

        // Define a box to add to the world.
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        collision_objects[0].primitives[0].dimensions[0] = 2.0;
        collision_objects[0].primitives[0].dimensions[1] = 2.0;
        collision_objects[0].primitives[0].dimensions[2] = 2.0;

        // Define the pose of the object
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 1.1;
        collision_objects[0].primitive_poses[0].position.y = 0.0;
        collision_objects[0].primitive_poses[0].position.z = -1.0;

        collision_objects[0].operation = collision_objects[0].ADD;


        // Ground back
        //____________________________________________________________________________
        // The id of the object is used to identify it.
        collision_objects[1].id = "my_back_ground";
        collision_objects[1].header.frame_id = "world";

        // Define a box to add to the world.
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[1].primitives[0].dimensions.resize(3);
        collision_objects[1].primitives[0].dimensions[0] = 2.0;
        collision_objects[1].primitives[0].dimensions[1] = 2.0;
        collision_objects[1].primitives[0].dimensions[2] = 2.0;

        // Define the pose of the object
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = -1.2;
        collision_objects[1].primitive_poses[0].position.y = 0.0;
        collision_objects[1].primitive_poses[0].position.z = -1.0;

        collision_objects[1].operation = collision_objects[1].ADD;


        // Ground right
        //____________________________________________________________________________
        // The id of the object is used to identify it.
        collision_objects[2].id = "my_right_ground";
        collision_objects[2].header.frame_id = "world";

        // Define a box to add to the world.
        collision_objects[2].primitives.resize(1);
        collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[2].primitives[0].dimensions.resize(3);
        collision_objects[2].primitives[0].dimensions[0] = 2.0;
        collision_objects[2].primitives[0].dimensions[1] = 2.0;
        collision_objects[2].primitives[0].dimensions[2] = 2.0;

        // Define the pose of the object
        collision_objects[2].primitive_poses.resize(1);
        collision_objects[2].primitive_poses[0].position.x = 0.0;
        collision_objects[2].primitive_poses[0].position.y = 1.2;
        collision_objects[2].primitive_poses[0].position.z = -1.0;

        collision_objects[2].operation = collision_objects[2].ADD;


        // Ground left
        //____________________________________________________________________________
        // The id of the object is used to identify it.
        collision_objects[3].id = "my_left_ground";
        collision_objects[3].header.frame_id = "world";

        // Define a box to add to the world.
        collision_objects[3].primitives.resize(1);
        collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[3].primitives[0].dimensions.resize(3);
        collision_objects[3].primitives[0].dimensions[0] = 2.0;
        collision_objects[3].primitives[0].dimensions[1] = 2.0;
        collision_objects[3].primitives[0].dimensions[2] = 2.0;

        // Define the pose of the object
        collision_objects[3].primitive_poses.resize(1);
        collision_objects[3].primitive_poses[0].position.x = 0.0;
        collision_objects[3].primitive_poses[0].position.y = -1.2;
        collision_objects[3].primitive_poses[0].position.z = -1.0;

        collision_objects[3].operation = collision_objects[3].ADD;

        
        // chessboard
        //____________________________________________________________________________
        // The id of the object is used to identify it.
        collision_objects[4].id = "my_chess_board";
        collision_objects[4].header.frame_id = "world";

        // Define a box to add to the world.
        collision_objects[4].primitives.resize(1);
        collision_objects[4].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[4].primitives[0].dimensions.resize(3);
        collision_objects[4].primitives[0].dimensions[0] = chessBoardDimension[0];
        collision_objects[4].primitives[0].dimensions[1] = chessBoardDimension[1];
        collision_objects[4].primitives[0].dimensions[2] = chessBoardDimension[2];

        // Define the pose of the object
        collision_objects[4].primitive_poses.resize(1);
        collision_objects[4].primitive_poses[0].position.x = chessBoardPosition[0];
        collision_objects[4].primitive_poses[0].position.y = chessBoardPosition[1];
        collision_objects[4].primitive_poses[0].position.z = chessBoardPosition[2];

        collision_objects[4].operation = collision_objects[4].ADD;


        // external camera
        //____________________________________________________________________________
        // The id of the object is used to identify it.
        collision_objects[5].id = "my_external_camera";
        collision_objects[5].header.frame_id = "world";

        // Define a box to add to the world.
        collision_objects[5].primitives.resize(1);
        collision_objects[5].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[5].primitives[0].dimensions.resize(3);
        collision_objects[5].primitives[0].dimensions[0] = externalCameraDimension[0];
        collision_objects[5].primitives[0].dimensions[1] = externalCameraDimension[1];
        collision_objects[5].primitives[0].dimensions[2] = externalCameraDimension[2];

        // Define the pose of the object
        collision_objects[5].primitive_poses.resize(1);
        collision_objects[5].primitive_poses[0].position.x = externalCameraPosition[0];
        collision_objects[5].primitive_poses[0].position.y = externalCameraPosition[1];
        collision_objects[5].primitive_poses[0].position.z = externalCameraPosition[2];

        collision_objects[5].operation = collision_objects[5].ADD;

        ROS_INFO("applying collision objects");
        planning_scene_interface_ptr->applyCollisionObjects(collision_objects);
    }

    


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // //////////////////// //
    // Internal Functions.  //
    // //////////////////// //

    // Function to publish to a specific sender with a supplied Protocol
    void sendToSender(BYTE sender, const ibo1_irc_api::Protocol& sendProtocol){

        cout << "I am sending to:" << (int)sender << endl;
        cout << "Following command: " << (int)sendProtocol.cmd << endl;

        // Checking which sender it should return to
        switch (sender)
        {
            case SENDER_SYSTEMSTATEMACHINE:{
                
                systemStateMachine_pub_ptr->publish(sendProtocol);
                break;
            }
            
            default:
                break;
        }
    }


    //Converts movement to CellPosition
    void getVecPosFromMove(string const &move, int &cellPosPickup, int &cellPosDrop){

        int rowMoveFrom = (int)move[0] - 97;
        int columnMoveFrom = (move[1] - '1');
        int rowMoveTo = (int)move[2] - 97;
        int columnMoveTo = (move[3] - '1');


        cellPosPickup = rowMoveFrom + (columnMoveFrom*8);
        cellPosDrop = rowMoveTo + (columnMoveTo*8);
    }



    // Setting the target cellPos
    bool getTargetCellsOccupied(vector<BYTE>& targetData, bool& pickUpOccupied, bool& dropOccupied){
        string moveCommand = "";
        DataCreator::convertBytesToString(targetData, moveCommand);

        transform(moveCommand.begin(), moveCommand.end(), moveCommand.begin(), ::tolower);

        if(DataChecker::isCorrectMoveFormat(moveCommand)){
            
            int targetCellPickUp = 0;
            int targetCellDrop = 0;

            getVecPosFromMove(moveCommand, targetCellPickUp, targetCellDrop);

            if (!returnedChessCells.chessCells.empty())
            {
                pickUpOccupied = returnedChessCells.chessCells.at(targetCellPickUp).isOccupied;
                dropOccupied = returnedChessCells.chessCells.at(targetCellDrop).isOccupied;
                return true;
            }
            

            return false;
        }

        return false;

    }

    bool updateTransform(const string& targetFrame, geometry_msgs::TransformStamped& transformToUpdate){

        // Deciding which targetet frame to go for
        // cout << "Frame to get transform:" << targetFrame << endl;
        
        try{
            transformToUpdate = tfBuffer_ptr->lookupTransform("base_link", targetFrame, ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }

        return true;

    }

    //Moving arm to specified frame
    bool moveArm( const string& targetFrame, const double speedValues[2]){

        geometry_msgs::TransformStamped transformToUpdatedTargetFrame;

        // Getting the transform to the specified target
        try{
            transformToUpdatedTargetFrame = tfBuffer_ptr->lookupTransform("base_link", targetFrame, ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }

        geometry_msgs::Pose goalPose;

        // Setting goalPose
        goalPose.position.x = transformToUpdatedTargetFrame.transform.translation.x;
        goalPose.position.y = transformToUpdatedTargetFrame.transform.translation.y;
        goalPose.position.z = transformToUpdatedTargetFrame.transform.translation.z;

        goalPose.orientation.x = transformToUpdatedTargetFrame.transform.rotation.x;
        goalPose.orientation.y = transformToUpdatedTargetFrame.transform.rotation.y;  
        goalPose.orientation.z = transformToUpdatedTargetFrame.transform.rotation.z;  
        goalPose.orientation.w = transformToUpdatedTargetFrame.transform.rotation.w;

        move_group_arm_ptr->setPoseTarget(goalPose);
        move_group_arm_ptr->setNumPlanningAttempts(10);
        move_group_arm_ptr->setPlanningTime(5.0);

        move_group_arm_ptr->setMaxVelocityScalingFactor(speedValues[0]);
        move_group_arm_ptr->setMaxAccelerationScalingFactor(speedValues[1]);
        

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


        if(success){
            return (move_group_gripper_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }
        
        return false;
    }

    // Gripper Controller
    bool gripperControl(const double& gripperValue){
        string joint = "finger_joint";

        move_group_gripper_ptr->setNumPlanningAttempts(10);
        move_group_gripper_ptr->setPlanningTime(5.0);
        move_group_gripper_ptr->setJointValueTarget(joint, gripperValue);
        move_group_gripper_ptr->setMaxVelocityScalingFactor(0.02);
        move_group_gripper_ptr->setMaxAccelerationScalingFactor(0.02);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_gripper_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success){
            
            return (move_group_gripper_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }

        return false;
    }


    bool arrivedAtPosition(const geometry_msgs::TransformStamped& targetPose){
        geometry_msgs::PoseStamped currentPose = move_group_arm_ptr->getCurrentPose();

        if(
            abs(targetPose.transform.translation.x - currentPose.pose.position.x) <= 0.025 &&
            abs(targetPose.transform.translation.y - currentPose.pose.position.y) <= 0.025 &&
            abs(targetPose.transform.translation.z - currentPose.pose.position.z) <= 0.025 &&
            abs(targetPose.transform.rotation.x - abs(currentPose.pose.orientation.x)) <= 0.005 &&
            abs(targetPose.transform.rotation.y - abs(currentPose.pose.orientation.y)) <= 0.005 &&
            abs(targetPose.transform.rotation.z - abs(currentPose.pose.orientation.z)) <= 0.005 &&
            abs(targetPose.transform.rotation.w - abs(currentPose.pose.orientation.w)) <= 0.005 
        ){
            return true;
        }
        else{
            return false;
        }
    }



    bool moveToTarget(const tf2::Quaternion rotation, geometry_msgs::TransformStamped& targetTransform, const double offSet[3], const double speedValues[2]){

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "updated_Target";


        transformStamped.transform.translation.x = targetTransform.transform.translation.x + offSet[0];
        transformStamped.transform.translation.y = targetTransform.transform.translation.y + offSet[1];
        transformStamped.transform.translation.z = targetTransform.transform.translation.z + offSet[2];
        tf2::Quaternion q;

        transformStamped.transform.rotation.x = rotation.x();
        transformStamped.transform.rotation.y = rotation.y();
        transformStamped.transform.rotation.z = rotation.z();
        transformStamped.transform.rotation.w = rotation.w();



        br.sendTransform(transformStamped);

        bool finishedMovement = moveArm("updated_Target", speedValues);
        if(finishedMovement && arrivedAtPosition(transformStamped)) return true;
        return false;

    }

    // Going to rest Position
    void moveToRestPosition(){
        double emptyOffset[3] = {0,0,0};

        //Going to rest position
        bool gotToRestPosition = false;
        while(!gotToRestPosition){
            ros::Duration(0.5).sleep();

            if(updateTransform(goalFrames[3], startFrameTransform)){
                gotToRestPosition = moveToTarget(quaternionPickDrop, startFrameTransform, emptyOffset, speedValuesArm[0]);
            }
            
        }

        gripperControl(0);
    }

    
    // Pick Up and Drop State stepper
    void pickUpAndDrop(double offSets[][3]){

        // Move back to rest Position set that we already updated TransformFrames
        if(pickDropState == 9){
            moveToRestPosition();
            robotArmState++;
            pickDropState = 0;
            transformFramesUpdated = false;
        }

        // Go back to intermediate Position for the frame to drop at
        else if(pickDropState == 8){
            if(moveToTarget(quaternionPickDrop, endFrameTransform, offSets[2], speedValuesArm[1])) pickDropState++;
        }

        // Let go of the Piece
        else if(pickDropState == 7){
            gripperFinished = gripperControl(gripperValues[0]);
            if(gripperFinished){
                pickDropState++;
                gripperFinished = false;
            }
        }

        //Move to Drop position
        else if(pickDropState == 6){
            if(moveToTarget(quaternionPickDrop, endFrameTransform, offSets[3], speedValuesArm[1])) pickDropState++;
        }
        
        // Move to intermediate Position for the frame to drop at
        else if(pickDropState == 5){
            if(moveToTarget(quaternionPickDrop, endFrameTransform, offSets[2], speedValuesArm[1])) pickDropState++;
        }

        // Move back up to intermediate Position
        else if(pickDropState == 4){
            if(moveToTarget(quaternionPickDrop,startFrameTransform,  offSets[0], speedValuesArm[1])) pickDropState++;
        }

        // Close the Grippers to grip the piece
        else if(pickDropState == 3){
            gripperFinished = gripperControl(gripperValues[1]);
            if(gripperFinished){
                pickDropState++;
                gripperFinished = false;
            }
        }

        // Move down to get the piece
        else if(pickDropState == 2){
            if(moveToTarget(quaternionPickDrop, startFrameTransform, offSets[1], speedValuesArm[1])) pickDropState++;
        }

        // Close Gripper slightly to get between chessPieces
        else if(pickDropState == 1){
            gripperFinished = gripperControl(gripperValues[0]);
            if(gripperFinished){
                pickDropState++;
                gripperFinished = false;
            }
        }

        // Move to intermediate Position
        else if(pickDropState == 0){
            if(moveToTarget(quaternionPickDrop, startFrameTransform, offSets[0], speedValuesArm[0])) pickDropState++;
        }
    }



    // Decides if we remove a piece or pick up a piece
    void makeMove(const bool& isRemovePiece){
        if(isRemovePiece){
            if(!transformFramesUpdated){
                bool updatedStartFrame = updateTransform(goalFrames[0], startFrameTransform);
                bool updateEndFrame = updateTransform(goalFrames[2], endFrameTransform);

                // If one of the transform updates failed return
                if(!updatedStartFrame || !updateEndFrame) return;

                transformFramesUpdated = true;
            }
            pickUpAndDrop(removePieceOffsets);
        }
        else{
            if(!transformFramesUpdated){
                bool updatedStartFrame = updateTransform(goalFrames[1], startFrameTransform);
                bool updateEndFrame = updateTransform(goalFrames[0], endFrameTransform);

                // If one of the transform updates failed return
                if(!updatedStartFrame || !updateEndFrame) return;

                transformFramesUpdated = true;
            }
            
            pickUpAndDrop(movePieceOffsets);
        }
        
    }


    // Main logic to run through
    void robotArmStateMachine(){

        cout << "RobotArmState: " << robotArmState << endl;
        cout << "PickDropState: " << pickDropState << endl;
        cout << "-------------------------------------------" << endl;

        

        // Waiting for move Command
        if(robotArmState == 0 && returnedProtocol.cmd == CMD_INTERNAL_ROBOTARMMOVE){
            
            // Getting the chessCells which we are trying to pickUp from and drop into
            bool pickUpCellOccupied = false;
            bool dropCellOccupied = false;

            // Checking if we could get the data, if not return and send error
            if(!getTargetCellsOccupied(returnedProtocol.data, pickUpCellOccupied, dropCellOccupied)){
                //Send something went wrong
                return;
            }

            // Checking if pickUp Cell is not occupied
            if(!pickUpCellOccupied){
                //Send error pickup cell not occupied and return
                return;
            }

            initialSender = returnedProtocol.sender;

            // Check if dropCell is occupied, if yes go to robotArmState 1,
            // as we need to clear the drop cell first
            if(dropCellOccupied){
                robotArmState = 1;
                pickDropState = 0;
            }

            // Otherwise we need can pick up and move
            else{
                robotArmState = 2;
                pickDropState = 0;
            }

        }

        // Clear the drop cell first
        else if(robotArmState == 1){
            makeMove(true);
        }

        // Pick up to move piece and move it to the cell
        else if(robotArmState == 2){
            makeMove(false);
        }

        // Finished move send finished command back
        else if(robotArmState == 3){
            ibo1_irc_api::Protocol sendProtocol;
            sendProtocol.cmd = CMD_INTERNAL_ROBOTARMMOVE;
            sendProtocol.sender = SENDER_ROBOTARMSTATEMACHINE;
            sendToSender(initialSender, sendProtocol);
            robotArmState = 0;
        }



        returnedProtocol.cmd == (BYTE)0x00;
    }



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "robotArmStateMachine");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber robotArmStateMachine_sub = nh.subscribe("/my_gen3/ircRobotArmStateMachine", 1, &robotArmStateMachineMessageReceived);
    ros::Subscriber chessCellDetection_sub = nh.subscribe("/imageProcessing/chessCellDetection", 1, &chessCellDetectionMessageReceived);

    ros::Publisher systemStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("/ircSystemStateMachine", 10);

    tf2_ros::Buffer tfBuffer;
    tfBuffer_ptr = &tfBuffer;

    tf2_ros::TransformListener tfListener(tfBuffer);

    systemStateMachine_pub_ptr = &systemStateMachine_pub;

    
    // Setting up the movegroup for arm and gripper
    bool foundRobotModel = false;

    // while(!foundRobotModel){
    //     try{
    //         ros::Duration(5).sleep();
    //         moveit::planning_interface::MoveGroupInterface move_group_arm("arm");
    //         move_group_arm_ptr = &move_group_arm;
    //         foundRobotModel = true;//Going to rest position
    // bool gotToRestPosition = false;
    // while(!gotToRestPosition){
    //     ros::Duration(3).sleep();
    //     ROS_INFO("Didnt get to rest position initial yet.");
    //     gotToRestPosition = moveArmToTarget(goalFrames[3]);
        
    // }
            
    //     }
    // }

    moveit::planning_interface::MoveGroupInterface move_group_arm("arm");
    move_group_arm_ptr = &move_group_arm;

    

    moveit::planning_interface::MoveGroupInterface move_group_gripper("gripper");
    move_group_gripper_ptr = &move_group_gripper;
    


    //Creating PlanningSceneInterface and setting up initial planning_scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/my_gen3");
    planning_scene_interface_ptr = &planning_scene_interface;
    settingUpPlanningInterface();


    moveToRestPosition();

    ros::Rate rate(10);

    while(ros::ok()){

        robotArmStateMachine();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}