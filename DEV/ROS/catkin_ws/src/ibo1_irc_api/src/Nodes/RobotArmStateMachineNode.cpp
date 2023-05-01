/*
 * RobotArmStateMachineNode - Ros node which is used for ircRobotArmStateMachine
 * <p>
 * This file describes the internal communication and execution of its respective commands.
 * It communicates with the ircSystemState machine to receive and respond with commands.
 * For how it is connected to the internal system refer to SystemStateMachineNode.cpp
 * 
 * The definition of the internal protocol is defined in the folder data/Protocol/InternalProtocol.h
 * 
 * <p>
 * This Node listens to the ChessBoardCellDetectionNode and uses that information check if a cell 
 * is occupied or not. It uses the frames set by CreateTarget to know where it needs to move it.
 * It has predefined move sets it goes through to pick and place chess pieces.
 * This Node uses the moveit interface to execute robot arm movements.
 * 
 * <p>
 * This Node provides following functionality:
 * 
 *  - Robot arm movement
 * 
 * <p>
 * This file is launched with the ircSystem.launch file
 * 
 * <p>
 * It uses the paramServer to get the systemDebug value if debugging is enabled or disabled
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see CMakeLists.txt
 * @see ircSystem.launch
 * @see SystemStateMachineNode.cpp
 * @see CreateTargetNode.cpp
 * @see ChessBoardCellDetectionNode.cpp
 * @see DataCreator.h
 * @see DataChecker.h
 * @see InternalProtocol.h
*/



    // ////////// //
    // Includes.  //
    // ////////// //

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


// Height offsets for removing a piece from the board
double chessPieceRemoveIntermediateStandardOffsetHeigh = 0.18;
double chessPieceRemovePickUpStandardOffsetHeigh = 0.012;
double chessPieceRemoveGraveYardIntermediateStandardOffsetHeigh = 0.18;
double chessPieceRemoveGraveYardDropStandardOffsetHeigh = 0.1;

// Height offsets for making a move forward
double chessPieceMoveIntermediateStandardOffsetHeight = 0.18;
double chessPieceMovePickUpStandardOffsetHeight = 0.012;
double chessPieceMoveDropIntermediateStandardOffsetHeight = 0.18;
double chessPieceMoveDropStandardOffsetHeigh = 0.09;

// Removing a piece offsets
double removePieceOffsets[4][3] = {
    {0, -0.0128, chessPieceRemoveIntermediateStandardOffsetHeigh},
    {0, -0.0128, chessPieceRemovePickUpStandardOffsetHeigh},
    {-0.2, 0, chessPieceRemoveGraveYardIntermediateStandardOffsetHeigh},
    {0, 0, chessPieceRemoveGraveYardDropStandardOffsetHeigh}
};

// Picking up a piece offsets
double movePieceOffsets[4][3] = {
    {0, -0.0128, chessPieceMoveIntermediateStandardOffsetHeight},
    {0, -0.0128, chessPieceMovePickUpStandardOffsetHeight},
    {0, -0.0128, chessPieceMoveDropIntermediateStandardOffsetHeight},
    {0, -0.0128, chessPieceMoveDropStandardOffsetHeigh},
};



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

// For debug
bool systemDebug = false;


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////// //
    // Callbacks. //
    // ////////// //

// Callback function for subscriber on /my_gen3/ircRobotArmStateMachine
void robotArmStateMachineMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;

    //Only prints if systemDebug true
    ROS_INFO_COND(systemDebug, "I received from: %d", returnedProtocol.sender);
    ROS_INFO_COND(systemDebug, "I received the CMD: %d", returnedProtocol.cmd);

}

//Callback on /imageProcessing/chessCellDetection
void chessCellDetectionMessageReceived(const ibo1_irc_api::ChessCells& msg){
    returnedChessCells = msg;
}

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ///////////////// //
    // Setup Functions.  //
    // ///////////////// //

    // Function to setup the planning interface
    // This includes creating the ground objects for collision and the camera and chessboard
    void settingUpPlanningInterface(){

        //Clearing previous planning scene objects
        planning_scene_interface_ptr->removeCollisionObjects(object_ids);


        ROS_INFO_COND(systemDebug, "Adding ground to planning scene");

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

        ROS_INFO_COND(systemDebug, "applying collision objects");
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
        
        //Only prints if systemDebug true
        ROS_INFO_COND(systemDebug, "I am sending to: %d", sender);
        ROS_INFO_COND(systemDebug, "I am sending cmd: %d", sendProtocol.cmd);

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


    //Converts chess movement to CellPosition
    void getVecPosFromMove(string const &move, int &cellPosPickup, int &cellPosDrop){

        int rowMoveFrom = (int)move[0] - 97;
        int columnMoveFrom = (move[1] - '1');
        int rowMoveTo = (int)move[2] - 97;
        int columnMoveTo = (move[3] - '1');


        cellPosPickup = rowMoveFrom + (columnMoveFrom*8);
        cellPosDrop = rowMoveTo + (columnMoveTo*8);
    }



    // Checks if the target cells are occupied and edits the supplied booleans
    // It returns false if it could not get the data needed
    bool getTargetCellsOccupied(vector<BYTE>& targetData, bool& pickUpOccupied, bool& dropOccupied){
        string moveCommand = "";
        DataCreator::convertBytesToString(targetData, moveCommand);

        transform(moveCommand.begin(), moveCommand.end(), moveCommand.begin(), ::tolower);

        ROS_INFO_COND(systemDebug, "Getting target cells for move: %s", moveCommand.c_str());

        // Checks if the movement command format is correct
        if(DataChecker::isCorrectMoveFormat(moveCommand)){
            
            int targetCellPickUp = 0;
            int targetCellDrop = 0;

            getVecPosFromMove(moveCommand, targetCellPickUp, targetCellDrop);

            // Checks if we have data from the ChessBoardCellDetectionNode
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


    // Updates the transform variable supplied to the specified targetFrame
    bool updateTransform(const string& targetFrame, geometry_msgs::TransformStamped& transformToUpdate){
        
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

        // Checking if we have successfully planned a movement
        bool success = (move_group_arm_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Checking if our plan was successful 
        if(success){
            // Try to execute the plan and return successful or not as true or false
            return (move_group_gripper_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }
        
        // If plan not successful return false
        return false;
    }


    // Used to control the grippers and put them to a specified value
    // This is specific to how the used Robot arm from Kinova works this most likely
    // needs to be rewritten if a different robot arm model is used
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
            
            bool finished = (move_group_gripper_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ros::Duration(0.5).sleep();
            return finished;
        }

        return false;
    }


    // Checks if we arrived at the supplied target pose
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


    // This Method tries to move to the supplied targetFrame with its supplied offset and rotation 
    bool moveToTarget(const tf2::Quaternion rotation, geometry_msgs::TransformStamped& targetTransform, const double offSet[3], const double speedValues[2]){

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        // Create our target frame and publish it
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


        // Trying to move the arm to the updated target
        bool finishedMovement = moveArm("updated_Target", speedValues);

        // Returns true or false depending on if we moved and we arrived at the position
        if(finishedMovement && arrivedAtPosition(transformStamped)) return true;
        return false;

    }


    // Method used to set the robot arm into the rest position
    void moveToRestPosition(){
        double emptyOffset[3] = {0,0,0};

        //Going to rest position
        bool gotToRestPosition = false;

        // Keeps going till we are at rest position
        while(!gotToRestPosition){
            ros::Duration(0.5).sleep();

            // Checks if our update transform to the restposition frame was correct
            if(updateTransform(goalFrames[3], startFrameTransform)){

                // If yes move to the target
                gotToRestPosition = moveToTarget(quaternionPickDrop, startFrameTransform, emptyOffset, speedValuesArm[0]);
            }
            
        }

        // Reset the gripper and set it to fully open
        gripperControl(0);
    }

    


    // Pick Up and Drop State stepper
    void pickUpAndDrop(double offSets[][3]){

        // Move back to rest Position set that we already updated TransformFrames
        if(pickDropState == 9){
            ROS_INFO_COND(systemDebug, "PickDropState: 9");
            moveToRestPosition();
            robotArmState++;
            pickDropState = 0;
            transformFramesUpdated = false;
        }

        // Go back to intermediate Position for the frame to drop at
        else if(pickDropState == 8){
            ROS_INFO_COND(systemDebug, "PickDropState: 8");
            if(moveToTarget(quaternionPickDrop, endFrameTransform, offSets[2], speedValuesArm[1])) pickDropState++;
        }

        // Let go of the Piece
        else if(pickDropState == 7){
            ROS_INFO_COND(systemDebug, "PickDropState: 7");
            gripperFinished = gripperControl(gripperValues[0]);
            if(gripperFinished){
                pickDropState++;
                gripperFinished = false;
            }
        }

        //Move to Drop position
        else if(pickDropState == 6){
            ROS_INFO_COND(systemDebug, "PickDropState: 6");
            if(moveToTarget(quaternionPickDrop, endFrameTransform, offSets[3], speedValuesArm[1])) pickDropState++;
        }
        
        // Move to intermediate Position for the frame to drop at
        else if(pickDropState == 5){
            ROS_INFO_COND(systemDebug, "PickDropState: 5");
            if(moveToTarget(quaternionPickDrop, endFrameTransform, offSets[2], speedValuesArm[1])) pickDropState++;
        }

        // Move back up to intermediate Position
        else if(pickDropState == 4){
            ROS_INFO_COND(systemDebug, "PickDropState: 4");
            if(moveToTarget(quaternionPickDrop,startFrameTransform,  offSets[0], speedValuesArm[1])) pickDropState++;
        }

        // Close the Grippers to grip the piece
        else if(pickDropState == 3){
            ROS_INFO_COND(systemDebug, "PickDropState: 3");
            gripperFinished = gripperControl(gripperValues[1]);
            if(gripperFinished){
                pickDropState++;
                gripperFinished = false;
            }
        }

        // Move down to get the piece
        else if(pickDropState == 2){
            ROS_INFO_COND(systemDebug, "PickDropState: 2");
            if(moveToTarget(quaternionPickDrop, startFrameTransform, offSets[1], speedValuesArm[1])) pickDropState++;
        }

        // Close Gripper slightly to get between chessPieces
        else if(pickDropState == 1){
            ROS_INFO_COND(systemDebug, "PickDropState: 1");
            gripperFinished = gripperControl(gripperValues[0]);
            if(gripperFinished){
                pickDropState++;
                gripperFinished = false;
            }
        }

        // Move to intermediate Position
        else if(pickDropState == 0){
            ROS_INFO_COND(systemDebug, "PickDropState: 0");
            if(moveToTarget(quaternionPickDrop, startFrameTransform, offSets[0], speedValuesArm[0])) pickDropState++;
        }
    }



    // Decides if we remove a piece or pick up a piece
    void makeMove(const bool& isRemovePiece){

        // Do we need to remove a piece
        if(isRemovePiece){

            // Checking if our transformFrames were updated
            if(!transformFramesUpdated){

                // Update to new transform frames
                bool updatedStartFrame = updateTransform(goalFrames[0], startFrameTransform);
                bool updateEndFrame = updateTransform(goalFrames[2], endFrameTransform);

                // If one of the transform updates failed return
                if(!updatedStartFrame || !updateEndFrame) return;

                ROS_INFO_COND(systemDebug, "Starting to remove a piece");
                transformFramesUpdated = true;
            }
            else{
                pickUpAndDrop(removePieceOffsets);
            }
            
        }
        // Otherwise we are trying to move a piece
        else{
            if(!transformFramesUpdated){
                bool updatedStartFrame = updateTransform(goalFrames[1], startFrameTransform);
                bool updateEndFrame = updateTransform(goalFrames[0], endFrameTransform);

                // If one of the transform updates failed return
                if(!updatedStartFrame || !updateEndFrame) return;
                ROS_INFO_COND(systemDebug, "Starting to move the piece");
                transformFramesUpdated = true;
            }
            else{
                pickUpAndDrop(movePieceOffsets);
            }
            
            
        }
        
    }


    // Main logic to run through
    void robotArmStateMachine(){
        BYTE gotCmd = returnedProtocol.cmd;
        

        // Waiting for move Command
        if(robotArmState == 0 && gotCmd == CMD_INTERNAL_ROBOTARMMOVE){

            ROS_INFO_COND(systemDebug, "We got the command to move");
            
            // Getting the chessCells which we are trying to pickUp from and drop into
            bool pickUpCellOccupied = false;
            bool dropCellOccupied = false;

            // Checking if we could get the data, if not return and send error
            if(!getTargetCellsOccupied(returnedProtocol.data, pickUpCellOccupied, dropCellOccupied)){
                //Send something went wrong
                ROS_WARN("Encountered an error for getTargetCellsOccupied -  This is not implemented yet!");
                return;
            }

            // Checking if pickUp Cell is not occupied
            if(!pickUpCellOccupied){
                //Send error pickup cell not occupied and return
                ROS_WARN("The cell to move from is not occupied. Mismatch between real chessboard and internal");

                // Create the error protocol and send it back to initial sender
                ibo1_irc_api::Protocol errorProtocol;
                errorProtocol.cmd = ERROR_INTERNAL_CMD_PICKUPCELLEMPTY;
                errorProtocol.sender = SENDER_ROBOTARMSTATEMACHINE;
                sendToSender(initialSender, errorProtocol);
                robotArmState = 0;
                return;
            }

            initialSender = returnedProtocol.sender;

            // Check if dropCell is occupied, if yes go to robotArmState 1,
            // as we need to clear the drop cell first
            if(dropCellOccupied){
                robotArmState = 1;
                pickDropState = 0;
                ROS_INFO_COND(systemDebug, "RobotArmState now state: 1");
            }

            // Otherwise we need can pick up and move
            else{
                robotArmState = 2;
                pickDropState = 0;
                ROS_INFO_COND(systemDebug, "RobotArmState now state: 2");
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
            ROS_INFO_COND(systemDebug, "RobotArmState now state: 3");

            // Prepare protocol to send back
            ibo1_irc_api::Protocol sendProtocol;
            sendProtocol.cmd = CMD_INTERNAL_ROBOTARMMOVE;
            sendProtocol.sender = SENDER_ROBOTARMSTATEMACHINE;
            sendToSender(initialSender, sendProtocol);
            robotArmState = 0;
        }

        returnedProtocol.cmd = (BYTE)0x00;
    }



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    // ROS node and spinner setup
    ros::init(argc, argv, "robotArmStateMachine");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    // Subscriber and publisher setup
    ros::Subscriber robotArmStateMachine_sub = nh.subscribe("/my_gen3/ircRobotArmStateMachine", 1, &robotArmStateMachineMessageReceived);
    ros::Subscriber chessCellDetection_sub = nh.subscribe("/imageProcessing/chessCellDetection", 1, &chessCellDetectionMessageReceived);

    ros::Publisher systemStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("/ircSystem/ircSystemStateMachine", 10);
    systemStateMachine_pub_ptr = &systemStateMachine_pub;

    // Transform listener setup
    tf2_ros::Buffer tfBuffer;
    tfBuffer_ptr = &tfBuffer;

    tf2_ros::TransformListener tfListener(tfBuffer);

    // Getting System debug param
    string systemDebugString = "";
    ros::param::param<string>("/systemDebug", systemDebugString, "false");

    // Trying to convert debug string to bool, if wrong we send a warning
    try{
        systemDebug = DataCreator::stringToBool(systemDebugString);
    } catch(invalid_argument e){
        // Printing warning message and setting to a default value of false
        ROS_WARN("Error converting debug param to bool: '%s' - set to default 'false'",e.what());
        systemDebug = false;
    }


    
    // Setting up the movegroup for arm and gripper
    bool gotToInitialPosition = false;

    // Initial transform
    geometry_msgs::TransformStamped initialTransform;
    initialTransform.transform.translation.x = 0.656;
    initialTransform.transform.translation.y = 0.002;
    initialTransform.transform.translation.z = 0.434;
    initialTransform.transform.rotation.x = 0.500;
    initialTransform.transform.rotation.y = 0.500;
    initialTransform.transform.rotation.z = 0.501;
    initialTransform.transform.rotation.w = 0.499;

    geometry_msgs::TransformStamped currentInitialPose;

    // Waiting till robotArm arrived at initial position
    while(!gotToInitialPosition){
        ros::Duration(0.5).sleep();
        updateTransform("tool_frame", currentInitialPose);
        
        // Checking if we arrived at initial position
        if(
            abs(initialTransform.transform.translation.x - currentInitialPose.transform.translation.x) <= 0.025 &&
            abs(initialTransform.transform.translation.y - currentInitialPose.transform.translation.y) <= 0.025 &&
            abs(initialTransform.transform.translation.z - currentInitialPose.transform.translation.z) <= 0.025 &&
            abs(initialTransform.transform.rotation.x - abs(currentInitialPose.transform.rotation.x)) <= 0.005 &&
            abs(initialTransform.transform.rotation.y - abs(currentInitialPose.transform.rotation.y)) <= 0.005 &&
            abs(initialTransform.transform.rotation.z - abs(currentInitialPose.transform.rotation.z)) <= 0.005 &&
            abs(initialTransform.transform.rotation.w - abs(currentInitialPose.transform.rotation.w)) <= 0.005 
        ){
            gotToInitialPosition = true;
        }
    }

    // Setting up MoveGroup Interface for arm and gripper
    moveit::planning_interface::MoveGroupInterface move_group_arm("arm");
    move_group_arm_ptr = &move_group_arm;

    moveit::planning_interface::MoveGroupInterface move_group_gripper("gripper");
    move_group_gripper_ptr = &move_group_gripper;
    


    //Creating PlanningSceneInterface and setting up initial planning_scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/my_gen3");
    planning_scene_interface_ptr = &planning_scene_interface;
    settingUpPlanningInterface();

    // Moving to rest position 
    moveToRestPosition();

    ros::Rate rate(10);

    while(ros::ok()){

        robotArmStateMachine();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}