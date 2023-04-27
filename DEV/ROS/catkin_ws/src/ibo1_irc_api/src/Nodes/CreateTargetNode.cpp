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
int targetCellPickUp = 0;
int targetCellDrop = 8;


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////// //
    // Callbacks. //
    // ////////// //
void createTargetMessageReceived(const ibo1_irc_api::Protocol& msg){
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

    // //////////////////// //
    // Internal Functions.  //
    // //////////////////// //

    //Creates the transform for the specified cell with its name and publishes
    void transformTarget(const int& cellPos, const string& transformName){
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "ec_depth_frame";
        transformStamped.child_frame_id = transformName;

        float depth = returnedChessCells.chessCells.at(cellPos).depth;
        float xPosChessCell = returnedChessCells.chessCells.at(cellPos).x;
        float yPosChessCell = returnedChessCells.chessCells.at(cellPos).y;

        float xOffset = (((xPosChessCell-(x_max/2))/focal)*depth);
        float yOffset = (((yPosChessCell-(y_max/2))/focal)*depth);

        transformStamped.transform.translation.x = xOffset;
        transformStamped.transform.translation.y = yOffset; //x offset
        transformStamped.transform.translation.z = depth; //y offset
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
        //If targetCell -1 then we need to stop publishing transform
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

        transform(moveCommand.begin(), moveCommand.end(), moveCommand.begin(), ::tolower);

        if(DataChecker::isCorrectMoveFormat(moveCommand)){
            getVecPosFromMove(moveCommand, targetCellPickUp, targetCellDrop);

            cout << "Setting Targets:" << endl;
            cout << "MoveCommand: " << moveCommand << endl;
            cout << "Pickup Cell: " << targetCellPickUp << endl;
            cout << "Drop Cell: " << targetCellDrop << endl;

            return true;
        }

        return false;

    }

    // Function to publish to a specific sender with a supplied Protocol
    void sendToSender(BYTE sender, const ibo1_irc_api::Protocol& sendProtocol){

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

    // Main logic to run through
    void createTargetLogic(){
        BYTE gotCMD = returnedProtocol.cmd;

        ibo1_irc_api::Protocol response;

        switch (gotCMD)
        {
            case CMD_INTERNAL_SETTARGET:{


                // Setting the new CellPositions
                if(setTarget(returnedProtocol.data)){
                    response.cmd = gotCMD;
                    response.data.push_back((BYTE)0x01);
                }
                else{
                    response.cmd = ERROR_INTERNAL_CMD_INVALIDMOVEFORMAT;
                }

                response.sender = SENDER_CREATETARGET;

                //Sending the response back to where the request came from
                sendToSender(returnedProtocol.sender, response);

                break;
            }

            case CMD_INTERNAL_CLEARTARGET:{
                //Resseting the targets
                targetCellPickUp = -1;
                targetCellDrop = -1;

                response.cmd = gotCMD;
                response.sender = SENDER_CREATETARGET;
                
                //Sending the response back to where the request came from
                sendToSender(returnedProtocol.sender, response);
                break;
            }
            
            default:
                break;
        }


        publishTransform();

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

    ros::Subscriber createTarget_sub = nh.subscribe("/ircCreateTarget", 1, &createTargetMessageReceived);
    ros::Subscriber chessCellDetection_sub = nh.subscribe("/imageProcessing/chessCellDetection", 1, &chessCellDetectionMessageReceived);

    ros::Publisher systemStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("ircSystemStateMachine", 10);

    systemStateMachine_pub_ptr = &systemStateMachine_pub;

    ros::Rate rate(20);

    while(ros::ok()){

        createTargetLogic();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}