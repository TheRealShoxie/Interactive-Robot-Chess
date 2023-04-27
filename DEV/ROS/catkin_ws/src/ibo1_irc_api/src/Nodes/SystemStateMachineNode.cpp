#include <ibo1_irc_api/SystemStateMachine/SystemStateMachineHelper.h>

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


// Overall StateMachine variables
int systemState = 0;

// StateMachine inSimulation
int inSimulationState = 0;

// StateMachine for chessEngineMove in Simulation
int inSimulationChessEngineMoveState = 0;
BYTE initialSenderMove = SENDER_SERVER;
BYTE initialSenderMoveCmd = (BYTE)0x00;
vector<BYTE> inSimulationDataStorage;


//chessEngineMove State machine variables


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

void systemStateMachineMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    cout << "----------------------------------------------------------" << endl;
    cout << "I received following on systemStateMachine: " << endl;
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


// StateMachine for simulation chess engine move
void inSimulationStateMachineChessEngineMove(){

    if(inSimulationChessEngineMoveState == 3){
        //Create the protocol for setting the targets
        ibo1_irc_api::Protocol sendProtocolToInitialSender;
        sendProtocolToInitialSender.cmd = initialSenderMoveCmd;
        sendProtocolToInitialSender.sender = SENDER_SYSTEMSTATEMACHINE;
        sendProtocolToInitialSender.data = inSimulationDataStorage;


        sendToSender(initialSenderMove, sendProtocolToInitialSender);
        inSimulationState = 0;
    }

    //Make the robot arm do the movement
    else if(inSimulationChessEngineMoveState == -2){
        
    }

    //ChessEngineMove state 2, waiting for CreateTargetNode to reply
    else if(inSimulationChessEngineMoveState == 2 && returnedProtocol.cmd != (BYTE)0x00){

        // Checking if the sender is not CreateTarget, then return
        if(returnedProtocol.sender != SENDER_CREATETARGET) return;

        // Checking if we got a incorrect Command back
        if(returnedProtocol.cmd != CMD_INTERNAL_SETTARGET){
            // Attaching the new sender of the message
            returnedProtocol.sender = SENDER_SYSTEMSTATEMACHINE;

            // Sending it back to the initial Sender
            sendToSender(initialSenderMove, returnedProtocol);

            // As a error occured we set the state back to idle for inSimulation
            inSimulationState = 0;
            return;

        }

        // Increase to next state for chessEngineMove
        inSimulationChessEngineMoveState++;
    }

    //ChessEngineMove state 1, sending set target with ChessMove to CreateTargetNode
    else if(inSimulationChessEngineMoveState == 1){

        //Create the protocol for setting the targets
        ibo1_irc_api::Protocol sendCreateTargetProtocol;
        sendCreateTargetProtocol.cmd = CMD_INTERNAL_SETTARGET;
        sendCreateTargetProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
        sendCreateTargetProtocol.data = inSimulationDataStorage;

        //Sending setTarget command to Create target
        sendToSender(SENDER_CREATETARGET, sendCreateTargetProtocol);

        //Increase to next state for chessEngineMove
        inSimulationChessEngineMoveState++;

    }

    // Waiting till we received return from chessEngine
    else if(inSimulationChessEngineMoveState == 0 && returnedProtocol.cmd != (BYTE)0x00){
        
        // Checking if the sender is not Chesswrapper, then return
        if(returnedProtocol.sender != SENDER_CHESSWRAPPER) return;

        // Checking if we got a incorrect Move back
        if(returnedProtocol.cmd != CMD_INTERNAL_CHESSENGINEMOVE){

            // Attaching the new sender of the message
            returnedProtocol.sender = SENDER_SYSTEMSTATEMACHINE;

            // Sending it back to the initial Sender
            sendToSender(initialSenderMove, returnedProtocol);

            // As a error occured we set the state back to idle for inSimulation
            inSimulationState = 0;
            return;
        }
        
        // Store the data temporarily
        inSimulationDataStorage = returnedProtocol.data;

        // Increase to next state for chessEngineMove
        inSimulationChessEngineMoveState++;
    }
}


// StateMachine for simulation playing a game of Chess
void inSimulationStateMachine(){

    // Checking if we are in going down the chessEngineMove state
    if(inSimulationState == 1){
        inSimulationStateMachineChessEngineMove();
    }

    // Checking if inSimulationState == 0 then we are in IDLE and waiting for chessEngine or playerMove
    else if(inSimulationState == 0){
        switch (returnedProtocol.cmd)
        {
            case CMD_INTERNAL_PLAYERMOVE:{
                //Getting the initial sender
                BYTE initialSender = returnedProtocol.sender;

                sendToSender(initialSender, SystemStateMachine::systemStateMachineChessEngineForwarder(returnedProtocol, chessWrapper_pub_ptr));
                break;
            }

            case CMD_INTERNAL_CHESSENGINEMOVE:{

                ibo1_irc_api::Protocol getChessEngineMoveProtocol;
                getChessEngineMoveProtocol.cmd = returnedProtocol.cmd;
                getChessEngineMoveProtocol.data = returnedProtocol.data;
                getChessEngineMoveProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                sendToSender(SENDER_CHESSWRAPPER, getChessEngineMoveProtocol);

                // Increasing the inSimulation state to 1.1 as we need to go down the chessEngineMove state
                inSimulationState = 1;
                inSimulationChessEngineMoveState = 0;
                initialSenderMove = returnedProtocol.sender;
                initialSenderMoveCmd = returnedProtocol.cmd;
                break;
            }

            case CMD_INTERNAL_STOPCHESSENGINE:{
                //Getting the initial sender
                BYTE initialSender = returnedProtocol.sender;

                sendToSender(initialSender, SystemStateMachine::systemStateMachineChessEngineForwarder(returnedProtocol, chessWrapper_pub_ptr));

                // After stopping the chessEngine we go back to initial idle state
                systemState = 0;
                break;
            }

            default:
                break;
        }
    }
}

// For IDLE state system is a messageForwarder
void messageForwarder(){
    BYTE gotCMD = returnedProtocol.cmd;

    switch (gotCMD)
    {
        case CMD_INTERNAL_GETCHESSENGINES:{
            //Getting the initial sender
            BYTE initialSender = returnedProtocol.sender;

            sendToSender(initialSender, SystemStateMachine::systemStateMachineChessEngineForwarder(returnedProtocol, chessWrapper_pub_ptr));
            break;
        }

        case CMD_INTERNAL_STARTCHESSENGINE:{
            //Getting the initial sender
            BYTE initialSender = returnedProtocol.sender;

            sendToSender(initialSender, SystemStateMachine::systemStateMachineChessEngineForwarder(returnedProtocol, chessWrapper_pub_ptr));

            //Moving to next state
            systemState = 1;
            break;
        }
    
        default:
            break;
    }
}


// Main State machine
void systemStateMachineLogic(){

    // Looking through all system states
    if(systemState == 1){

        // Checking if we need to execute simulation path
        if(isSimulation){
            inSimulationStateMachine();
        }
        // Otherwise do inperson path
        else{
            cout << "Not Implemented!" << endl;
        }
    }
    else if(systemState == 0){
        messageForwarder();
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

    ros::Subscriber systemStateMachine_sub = nh.subscribe("/ircSystemStateMachine", 1, &systemStateMachineMessageReceived);

    ros::Publisher server_pub = nh.advertise<ibo1_irc_api::Protocol>("ircServer", 10);
    ros::Publisher chessWrapper_pub = nh.advertise<ibo1_irc_api::Protocol>("ircChessWrapper", 10);
    ros::Publisher createTarget_pub = nh.advertise<ibo1_irc_api::Protocol>("ircCreateTarget", 10);

    server_pub_ptr = &server_pub;
    chessWrapper_pub_ptr = &chessWrapper_pub;
    createTarget_pub_ptr = &createTarget_pub;

    ros::Rate rate(10);


    while(ros::ok()){
        systemStateMachineLogic();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}