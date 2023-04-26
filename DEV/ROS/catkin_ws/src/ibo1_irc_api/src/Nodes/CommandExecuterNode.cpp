#include <ibo1_irc_api/CommandExecuter/CommandExecuterHelper.h>

using namespace std;
/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //

// For ros subscribers & publisher
ibo1_irc_api::Protocol returnedProtocol;
ros::Publisher* server_pub_ptr;
ros::Publisher* chessWrapper_pub_ptr;
ros::Publisher* createTarget_pub_ptr;

// Command Executer variables
static const bool isSimulation = true;

// Command State
int commandState = 0;
bool commandAlreadyReceived;

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

void commandExecuterMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    cout << "----------------------------------------------------------" << endl;
    cout << "I received following on commandExecuter: " << endl;
    cout << "I received from: " << (int)returnedProtocol.sender << endl;
    cout << "CmdByte: " << (int)returnedProtocol.cmd << endl;
    cout << "----------------------------------------------------------" << endl;
}




/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // //////////////////// //
    // Internal Functions.  //
    // //////////////////// //

// Function to publish to a specific sender with a supplied Protocol
void sendToSender(BYTE sender, const ibo1_irc_api::Protocol& sendProtocol){

    // Checking which sender it should return to
    switch (sender)
    {
        case SENDER_SERVER:{
            server_pub_ptr->publish(sendProtocol);
            break;
        }
        case SENDER_CHESSWRAPPER:{
            chessWrapper_pub_ptr->publish(sendProtocol);
            break;
        }
        case SENDER_CREATETARGET:{
            createTarget_pub_ptr->publish(sendProtocol);
        }
        
        default:
            break;
    }
}

//State machine for making a chessEngine move
void stateMachineChessEngineMove(){
    //Getting the initial sender
    BYTE initialSender = returnedProtocol.sender;
    
    // Creating the chessEngine move
    ibo1_irc_api::Protocol protocolFromChessWrapper = CommandExecuterHelper::chessEngineForwarder(returnedProtocol, chessWrapper_pub_ptr);

    if(protocolFromChessWrapper.cmd != CMD_INTERNAL_CHESSENGINEMOVE){
        sendToSender(initialSender, protocolFromChessWrapper);
        return;
    }

    // Now I need to allocate the target at the position
    ibo1_irc_api::Protocol protocolFromTargetAllocation = CommandExecuterHelper::commandExecuterForwarder(returnedProtocol, SENDER_CHESSWRAPPER, chessWrapper_pub_ptr);
    
     

    //sendToSender(initialSender, CommandExecuterHelper::commandExecuterForwarder(returnedProtocol, SENDER_CHESSWRAPPER, chessWrapper_pub_ptr));
    return;
}


// Logic for simulation
void stateMachineLogicSimulation(){
    ibo1_irc_api::Protocol sendProtocol;

    BYTE gotCMD = returnedProtocol.cmd;

    switch (gotCMD)
    {

        case CMD_INTERNAL_GETCHESSENGINES:{
            
        }

        case CMD_INTERNAL_STARTCHESSENGINE:{
            //Getting the initial sender
            BYTE initialSender = returnedProtocol.sender;

            sendToSender(initialSender, CommandExecuterHelper::chessEngineForwarder(returnedProtocol, chessWrapper_pub_ptr));
            break;
        }

        case CMD_INTERNAL_STOPCHESSENGINE:{
            //Getting the initial sender
            BYTE initialSender = returnedProtocol.sender;

            sendToSender(initialSender, CommandExecuterHelper::chessEngineForwarder(returnedProtocol, chessWrapper_pub_ptr));
            break;
        }

        case CMD_INTERNAL_PLAYERMOVE:{
            //Getting the initial sender
            BYTE initialSender = returnedProtocol.sender;

            sendToSender(initialSender, CommandExecuterHelper::chessEngineForwarder(returnedProtocol, chessWrapper_pub_ptr));
            break;
        }

        case CMD_INTERNAL_CHESSENGINEMOVE:{
            BYTE initialSender = returnedProtocol.sender;

            sendToSender(initialSender, CommandExecuterHelper::chessEngineForwarder(returnedProtocol, chessWrapper_pub_ptr));
            break;
        }

        default:
            break;
    
    }


    returnedProtocol.cmd = (BYTE)0x00; 
}

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "commandExecuter");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber server_sub = nh.subscribe("/ircCommandExecuter", 1, &commandExecuterMessageReceived);

    ros::Publisher server_pub = nh.advertise<ibo1_irc_api::Protocol>("ircServer", 10);
    ros::Publisher chessWrapper_pub = nh.advertise<ibo1_irc_api::Protocol>("ircChessWrapper", 10);
    ros::Publisher createTarget_pub = nh.advertise<ibo1_irc_api::Protocol>("ircCreateTarget", 10);

    server_pub_ptr = &server_pub;
    chessWrapper_pub_ptr = &chessWrapper_pub;
    createTarget_pub_ptr = &createTarget_pub;

    ros::Rate rate(10);


    while(ros::ok()){
        if(isSimulation){
            stateMachineLogicSimulation();
        }
        else{
            cout << "Not implemented!!" << endl;
        }
        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}