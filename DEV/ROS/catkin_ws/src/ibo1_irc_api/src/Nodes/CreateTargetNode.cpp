#include <ros/ros.h>

#include <ibo1_irc_api/Protocol.h>
#include <ibo1_irc_api/ChessCells.h>
#include <ibo1_irc_api/ProtocolAPI/InternalProtocolDefinition.h>

#include <string>


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //
// Image sizes
static const int x_max = 1280;
static const int y_max = 720;

// For ros subscribers & publisher
ibo1_irc_api::Protocol returnedProtocol;
ibo1_irc_api::ChessCells returnedChessCells;

ros::Publisher* commandExecuter_pub_ptr;

// Target cell
int targetCell = -1;


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
    cout << "----------------------------------------------------------" << endl;
    cout << "I received following on createTarget: " << endl;
    cout << "----------------------------------------------------------" << endl;
}



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // //////////////////// //
    // Internal Functions.  //
    // //////////////////// //

    void publishTransform(){
        //If targetCell -1 then we need to stop publishing transform
        if(tragetCell == -1) return;

        

    }

    void getVecPosFromMove(string const &move, int &cellPos){

        int rowMoveFrom = (int)move[0] - 97;
        int columnMoveFrom = 7 - (move[1] - '1');

        cellPos = columnMoveFrom + (rowMoveFrom*8);
    }

    // Setting the target
    void setTarget(vector<BYTE>& target){


        string moveCommand = "";
        DataCreator::convertBytesToString(target, moveCommand);

        getVecPosFromMove(moveCommand, targetCell);

    }



    void createTargetLogic(){
        BYTE gotCMD = returnedProtocol.cmd;

        switch (gotCMD)
        {
        case CMD_INTERNAL_SETTARGET:
            setTarget(returnedProtocol.data);
            break;
        
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
    ros::Subscriber chessCellDetection_sub = nh.subscribe("/chessCellDetection", 1, &chessCellDetectionMessageReceived);

    ros::Publisher commandExecuter_pub = nh.advertise<ibo1_irc_api::Protocol>("ircCommandExecuter", 10);

    commandExecuter_pub_ptr = &commandExecuter_pub;

    ros::Rate rate(10);

    while(ros::ok()){

        createTargetLogic();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}