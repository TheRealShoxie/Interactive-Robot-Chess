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
static const float chessBoardPosition[3] = {-0.25, 0.5125, 0.018};
static const float chessBoardDimension[3] = {0.46, 0.46, 0.04};
static const float externalCameraPosition[3] = {-0.25, 0.5125, 1};
static const float externalCameraDimension[3] = {0.05, 0.05, 0.025};


//Robot Arm State Machine Variables
int robotArmState = 1;
int pickDropState = 0;
BYTE initialSender = (BYTE)0x00;
int cellPosPickup = -1;
int cellPosDrop = -1;


// Transform global variables

tf2_ros::Buffer* tfBuffer_ptr;
geometry_msgs::TransformStamped targetTransform;


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////// //
    // Callbacks. //
    // ////////// //
void robotArmStateMachineMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    cout << "----------------------------------------------------------" << endl;
    cout << "I received following on createTarget: " << endl;
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

    void updateTransformToTarget(const bool& isPickUpCell){

        // Deciding which targetet frame to go for
        string target = "";
        if(isPickUpCell){
            target = "target_pickup_frame";
        }
        else{
            target = "target_drop_frame";
        }


        try{
            targetTransform = tfBuffer_ptr->lookupTransform("base_link", target, ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }


    }

    void moveToPiece(){

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;


        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "updated_Target";


        transformStamped.transform.translation.x = targetTransform.transform.translation.x + 0.05;
        transformStamped.transform.translation.y = targetTransform.transform.translation.y + 0.05;
        transformStamped.transform.translation.z = targetTransform.transform.translation.z + 0.05;
        tf2::Quaternion q;

        //setRPY(roll, pitch, yaw) calculates quaternion rotation for transform
        q.setRPY(-M_PI/2, M_PI, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();



        br.sendTransform(transformStamped);
    }

    
    // Pick Up and Drop State stepper
    void pickUpAndDrop(const bool& isPickUpCell){

        if(pickDropState == 4){

        }

        else if(pickDropState == 3){

        }

        else if(pickDropState == 2){

        }

        else if(pickDropState == 1){

        }

        else if(pickDropState == 0){
            updateTransformToTarget(isPickUpCell);
            cout << targetTransform << endl;
            moveToPiece();
        }
    }


    // Main logic to run through
    void robotArmStateMachine(){

        // Pick up to move piece and drop it
        if(robotArmState == 2){
            pickUpAndDrop(true);
        }

        // Clear the drop cell first
        if(robotArmState == 1){
            pickUpAndDrop(false);
        }

        // Waiting for move Command
        else if(robotArmState == 0 && returnedProtocol.cmd == CMD_INTERNAL_ROBOTARMMOVE){
            
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

    ros::Subscriber robotArmStateMachine_sub = nh.subscribe("/ircRobotArmStateMachine", 1, &robotArmStateMachineMessageReceived);
    ros::Subscriber chessCellDetection_sub = nh.subscribe("/imageProcessing/chessCellDetection", 1, &chessCellDetectionMessageReceived);

    ros::Publisher systemStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("ircSystemStateMachine", 10);

    tf2_ros::Buffer tfBuffer;
    tfBuffer_ptr = &tfBuffer;

    tf2_ros::TransformListener tfListener(tfBuffer);



    systemStateMachine_pub_ptr = &systemStateMachine_pub;


    //Creating PlanningSceneInterface and setting up initial planning_scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/my_gen3");
    planning_scene_interface_ptr = &planning_scene_interface;
    settingUpPlanningInterface();

    ros::Rate rate(10);

    while(ros::ok()){

        robotArmStateMachine();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}