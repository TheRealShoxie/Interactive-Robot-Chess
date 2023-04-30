/*
 * CreateTargetNode - Ros node which is used for ircCreateTarget
 * <p>
 * This file describes the internal communication and execution of its respective commands.
 * It communicates with the ircSystemState machine to receive and respond with commands.
 * For how it is connected to the internal system refer to SystemStateMachineNode.cpp
 * 
 * The definition of the internal protocol is defined in the folder data/Protocol/InternalProtocol.h
 * 
 * <p>
 * This Node listens to the ChessBoardCellDetectionNode and uses that information to create frames for the
 * RobotArmStateMachine to use. Set and clear targets enable the setting of where the frame is supposed to 
 * be placed and if it needs to be removed(Stops publishing the frame)
 * 
 * 
 * <p>
 * This Node provides following functionality:
 * 
 *  - Set a target
 *  - Clear a target
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
 * @see ChessBoardCellDetectionNode.cpp
 * @see RobotArmStateMachineNode.cpp
 * @see InternalProtocol.h
 * @see DataCreator.h
 * @see DataChecker.h
*/


    // ////////// //
    // Includes.  //
    // ////////// //
    
#include <ros/ros.h>


// Protocol includes
#include <ibo1_irc_api/Protocol.h>
#include <ibo1_irc_api/ProtocolAPI/InternalProtocolDefinition.h>


// Transform includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// Self defined messages include
#include <ibo1_irc_api/ChessCells.h>

// Own header includes
#include <ibo1_irc_api/Utility/DataCreator.h>
#include <ibo1_irc_api/Utility/DataChecker.h>

//C++ includes
#include <string>


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //
// Image sizes
static const float x_max = 1280;
static const float y_max = 720;
static const float focal = 695.9951364338076;

// For ros subscribers & publisher
ibo1_irc_api::Protocol returnedProtocol;
ibo1_irc_api::ChessCells returnedChessCells;

ros::Publisher* systemStateMachine_pub_ptr;

// Target cell
int targetCellPickUp = -1;
int targetCellDrop = -1;

// For debug
bool systemDebug = false;


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////// //
    // Callbacks. //
    // ////////// //

// Callback function for subscriber on /ircSystem/ircCreateTarget
void createTargetMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    //Only prints if systemDebug true
    ROS_INFO_COND(systemDebug, "I received from: %d", returnedProtocol.sender);
    ROS_INFO_COND(systemDebug, "I received the CMD: %d", returnedProtocol.cmd);
}

// Callback function for subscriber on /imageProcessing/chessCellDetection to listen for published cells
void chessCellDetectionMessageReceived(const ibo1_irc_api::ChessCells& msg){
    returnedChessCells = msg;
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

    //Creates the transform for the specified cell with its name and publishes
    void transformTarget(const int& cellPos, const string& transformName){
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "ec_depth_frame";
        transformStamped.child_frame_id = transformName;

        // Getting the x,y and depth information from chessBoardCellDetectionNode
        float depth = returnedChessCells.chessCells.at(cellPos).depth;
        float xPosChessCell = returnedChessCells.chessCells.at(cellPos).x;
        float yPosChessCell = returnedChessCells.chessCells.at(cellPos).y;

        // Calculate the x and y offset based on camera
        float xOffset = (((xPosChessCell-(x_max/2))/focal)*depth);
        float yOffset = (((yPosChessCell-(y_max/2))/focal)*depth);

        // Setting the transform translation x,y,z
        transformStamped.transform.translation.x = xOffset;
        transformStamped.transform.translation.y = yOffset;
        transformStamped.transform.translation.z = depth;
        tf2::Quaternion q;

        //setRPY(roll, pitch, yaw) calculates quaternion rotation for transform
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
    }


    // Calls transform target to publish the targets and does some checking
    void publishTransform(){

        // If targetCell -1 then we need to stop publishing transform
        // As target was set to clear or error occured
        if(targetCellPickUp == -1 || targetCellDrop == -1) return;

        //Checking if we have empty chessCell information
        if(returnedChessCells.chessCells.empty()) return;

        //Transforms the target and publishes
        transformTarget(targetCellPickUp, "target_pickup_frame");
        transformTarget(targetCellDrop, "target_drop_frame");
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

    // Setting the target
    bool setTarget(vector<BYTE>& targetData){
        string moveCommand = "";
        DataCreator::convertBytesToString(targetData, moveCommand);

        // Convert moveCmd to lower case
        transform(moveCommand.begin(), moveCommand.end(), moveCommand.begin(), ::tolower);

        // Checks if we have a correct moveCommand
        if(DataChecker::isCorrectMoveFormat(moveCommand)){

            getVecPosFromMove(moveCommand, targetCellPickUp, targetCellDrop);
            ROS_INFO_COND(systemDebug, "I set target for moveCommand: %s", moveCommand.c_str());

            return true;
        }

        return false;

    }

    // Main logic to run through
    void createTargetLogic(){
        BYTE gotCMD = returnedProtocol.cmd;

        ibo1_irc_api::Protocol response;

        // Checking the cmd we got
        switch (gotCMD)
        {

            // Setting a target
            case CMD_INTERNAL_SETTARGET:{
                // Setting the new CellPositions and checking if successful
                if(setTarget(returnedProtocol.data)){
                    response.cmd = gotCMD;
                    response.data.push_back(CMD_INTERNAL_SETTARGET);
                }

                // Otherwise moveFormat wasnt correct
                else{
                    response.cmd = ERROR_INTERNAL_CMD_INVALIDMOVEFORMAT;
                    ROS_WARN_COND(systemDebug, "MoveFormat incorrect: %d", returnedProtocol.sender);
                }


                // Attaching the sender
                response.sender = SENDER_CREATETARGET;

                //Sending the response back to where the request came from
                sendToSender(returnedProtocol.sender, response);

                break;
            }


            // Clearing target
            case CMD_INTERNAL_CLEARTARGET:{


                ROS_INFO_COND(systemDebug, "Clearing target");

                //Resseting the targets
                targetCellPickUp = -1;
                targetCellDrop = -1;

                response.cmd = CMD_INTERNAL_CLEARTARGET;
                response.sender = SENDER_CREATETARGET;
                
                //Sending the response back to where the request came from
                sendToSender(returnedProtocol.sender, response);
                break;
            }
            
            default:
                break;
        }

        // Continiously publish the transform
        publishTransform();

        // Resets returnedProtocol.cmd
        returnedProtocol.cmd = (BYTE)0x00; 
    }



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "createTarget");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    // Subscriber and publisher setup
    ros::Subscriber createTarget_sub = nh.subscribe("/ircSystem/ircCreateTarget", 1, &createTargetMessageReceived);
    ros::Subscriber chessCellDetection_sub = nh.subscribe("/imageProcessing/chessCellDetection", 1, &chessCellDetectionMessageReceived);

    ros::Publisher systemStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("ircSystemStateMachine", 10);

    systemStateMachine_pub_ptr = &systemStateMachine_pub;

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

    ros::Rate rate(20);

    while(ros::ok()){

        createTargetLogic();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}