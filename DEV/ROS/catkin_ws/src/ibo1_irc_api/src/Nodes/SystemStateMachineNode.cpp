#include <ibo1_irc_api/SystemStateMachine/SystemStateMachineHelper.h>

#include <ibo1_irc_api/Utility/DataCreator.h>

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
ros::Publisher* robotArmStateMachine_pub_ptr;

// Command Executer variables
static const bool isSimulation = true;


// Overall StateMachine variables
int systemState = 0;

// StateMachine inSimulation
int inSimulationState = 0;

// StateMachine for chessEngineMove in Simulation
int inSimulationChessEngineMoveState = 0;
int inSimulationPlayerMoveState = 0;
BYTE initialSenderMove = SENDER_SERVER;
BYTE initialSenderMoveCmd = (BYTE)0x00;
vector<BYTE> inSimulationDataStorage;


//Variable for castling checking
bool didCastle = false;
vector<BYTE> inSimulationDataStorageForCastling;


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
                break;
            }

            case SENDER_ROBOTARMSTATEMACHINE:{
                robotArmStateMachine_pub_ptr->publish(sendProtocol);
                break;
            }
            
            default:
                break;
        }
    }

    // Get the second move of castle move
    string secondMoveOfCastleMove(){
        
        string moveCmd = "";
        DataCreator::convertBytesToString(inSimulationDataStorage, moveCmd);

        //White Castle King side
        if(moveCmd.compare("e1g1") == 0){
            inSimulationDataStorageForCastling = inSimulationDataStorage;
            didCastle = true;
            string newMoveCmd = "h1f1";
            copy(newMoveCmd.begin(), newMoveCmd.end(), std::back_inserter(inSimulationDataStorage));
        }
        //White Castle Queen side
        else if(moveCmd.compare("e1c1") == 0){
            inSimulationDataStorageForCastling = inSimulationDataStorage;
            didCastle = true;
            string newMoveCmd = "a1d1";
            copy(newMoveCmd.begin(), newMoveCmd.end(), std::back_inserter(inSimulationDataStorage));
        }
        //Black Castle King side
        else if(moveCmd.compare("e8g8") == 0){
            inSimulationDataStorageForCastling = inSimulationDataStorage;
            didCastle = true;
            string newMoveCmd = "h8f8";
            copy(newMoveCmd.begin(), newMoveCmd.end(), std::back_inserter(inSimulationDataStorage));
        }
        //Black Castle Queen side
        else if(moveCmd.compare("e8c8") == 0){
            inSimulationDataStorageForCastling = inSimulationDataStorage;
            didCastle = true;
            string newMoveCmd = "a8d8";
            copy(newMoveCmd.begin(), newMoveCmd.end(), std::back_inserter(inSimulationDataStorage));
        }
    }
    
    // in Simulation States for setting target moving robot arm and clearing target
    void inSimulationRobotMove(int &inSimulationRobotMoveState){

        // Waiting till ChessWrapper send back is castle move
        if(inSimulationRobotMoveState == 5 && returnedProtocol.cmd != (BYTE)0x00){

            //Checking if the sender is not ChessWrapper, then return
            if(returnedProtocol.sender != SENDER_CHESSWRAPPER) return;

            //Checking if we got an incorrect Command back
            if(returnedProtocol.cmd != CMD_INTERNAL_LASTMOVECASTLEMOVE){

                // Checking if we didnt get back last move not castle move error
                if(returnedProtocol.cmd != ERROR_INTERNAL_CMD_LASTMOVEWASNOTCASTLEMOVE){

                    //Attaching the new sender of the message
                    returnedProtocol.sender = SENDER_SYSTEMSTATEMACHINE;

                    // Sending it back to the initial Sender
                    sendToSender(initialSenderMove, returnedProtocol);

                    // As a error occured we set the state back to idle for inSimulation
                    inSimulationState = 0;
                    return;
                }

                // As we got back the not castle move we increase the inSimulationRobotMoveState
                inSimulationRobotMoveState++;
            }

            // As we got information that last move was castle move we need to update the 
        }

        // Waiting till we cleared the target then we go into next stage
        if(inSimulationRobotMoveState == 4 && returnedProtocol.cmd != (BYTE)0x00){
            // Checking if the sender is not CreateTarget, then return
            if(returnedProtocol.sender != SENDER_CREATETARGET) return;

            // Checking if we got a incorrect Command back
            if(returnedProtocol.cmd != CMD_INTERNAL_CLEARTARGET){
                // Attaching the new sender of the message
                returnedProtocol.sender = SENDER_SYSTEMSTATEMACHINE;

                // Sending it back to the initial Sender
                sendToSender(initialSenderMove, returnedProtocol);

                // As a error occured we set the state back to idle for inSimulation
                inSimulationState = 0;
                return;

            }

            //Create the protocol for checking if lastMove was a castle move
            ibo1_irc_api::Protocol sendChessWrapperProtocol;
            sendChessWrapperProtocol.cmd = CMD_INTERNAL_LASTMOVECASTLEMOVE;
            sendChessWrapperProtocol.sender = SENDER_SYSTEMSTATEMACHINE;

            //Sending setTarget command to Create target
            sendToSender(SENDER_CHESSWRAPPER, sendChessWrapperProtocol);

            // Checking if we just did a castle move
            // If reset didCastle and skip check for castling move and set internalSimulationDataStorage back to original
            if(didCastle){
                
                // Reset didCastle
                didCastle = false;

                // Increase to next state twice as we already did castle check
                inSimulationRobotMoveState = inSimulationRobotMoveState + 2;

                //Setting original data back into inSimulation and clearing temp castling data to free up memory
                inSimulationDataStorage = inSimulationDataStorageForCastling;
                inSimulationDataStorageForCastling.clear();

            }
            // Otherwise just increase inSimulationRobotMoveState
            else{
                // Increase to next state for chessEngineMove
                inSimulationRobotMoveState++;
            }
        }

        //Waiting till robot arm finished moving
        else if(inSimulationRobotMoveState == 3 && returnedProtocol.cmd != (BYTE)0x00){

            // Checking if the sender is not RobotArmStateMachine, then return
            if(returnedProtocol.sender != SENDER_ROBOTARMSTATEMACHINE) return;

            // Checking if we got a incorrect Command back
            if(returnedProtocol.cmd != CMD_INTERNAL_ROBOTARMMOVE){

                // Attaching the new sender of the message
                returnedProtocol.sender = SENDER_SYSTEMSTATEMACHINE;

                // Sending it back to the initial Sender
                sendToSender(initialSenderMove, returnedProtocol);

                // As a error occured we set the state back to idle for inSimulation
                inSimulationState = 0;
                return;
            }

            //Create the protocol for setting the targets
            ibo1_irc_api::Protocol sendResetTargetProtocol;
            sendResetTargetProtocol.cmd = CMD_INTERNAL_CLEARTARGET;
            sendResetTargetProtocol.sender = SENDER_SYSTEMSTATEMACHINE;

            //Sending setTarget command to Create target
            sendToSender(SENDER_CREATETARGET, sendResetTargetProtocol);

            //Increase to next state for chessEngineMove
            inSimulationRobotMoveState++;

        }

        //ChessEngineMove state 2, waiting for CreateTargetNode to reply
        else if(inSimulationRobotMoveState == 2 && returnedProtocol.cmd != (BYTE)0x00){


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

            // Now that we set the target we want to have the robnot arm do its movements
            // Create the protocol for the robot arm
            ibo1_irc_api::Protocol sendRobotArmMoveProtocol;
            sendRobotArmMoveProtocol.cmd = CMD_INTERNAL_ROBOTARMMOVE;
            sendRobotArmMoveProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
            sendRobotArmMoveProtocol.data = inSimulationDataStorage;

            // Sending RobotArmMove command to robotArmStateMachineNode
            sendToSender(SENDER_ROBOTARMSTATEMACHINE, sendRobotArmMoveProtocol);

            // Increase to next state for chessEngineMove
            inSimulationRobotMoveState++;
        }

        //ChessEngineMove state 1, sending set target with ChessMove to CreateTargetNode
        else if(inSimulationRobotMoveState == 1){

            //Create the protocol for setting the targets
            ibo1_irc_api::Protocol sendCreateTargetProtocol;
            sendCreateTargetProtocol.cmd = CMD_INTERNAL_SETTARGET;
            sendCreateTargetProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
            sendCreateTargetProtocol.data = inSimulationDataStorage;

            //Sending setTarget command to Create target
            sendToSender(SENDER_CREATETARGET, sendCreateTargetProtocol);

            //Increase to next state for chessEngineMove
            inSimulationRobotMoveState++;
        }
    }


    // StateMachine for simulation chess engine move
    void inSimulationPlayerMove(){

        if(inSimulationPlayerMoveState == 6){
            //Create the protocol for sending back to the initial sender
            ibo1_irc_api::Protocol sendProtocolToInitialSender;
            sendProtocolToInitialSender.cmd = initialSenderMoveCmd;
            sendProtocolToInitialSender.sender = SENDER_SYSTEMSTATEMACHINE;
            sendProtocolToInitialSender.data = inSimulationDataStorage;


            sendToSender(initialSenderMove, sendProtocolToInitialSender);
            inSimulationState = 0;
        }

        // Waiting till we received return from ChessWrapper
        else if(inSimulationPlayerMoveState == 0 && returnedProtocol.cmd != (BYTE)0x00){
            
            // Checking if the sender is not Chesswrapper, then return
            if(returnedProtocol.sender != SENDER_CHESSWRAPPER) return;

            // Checking if we got a incorrect Move back
            if(returnedProtocol.cmd != CMD_INTERNAL_PLAYERMOVE){

                // Attaching the new sender of the message
                returnedProtocol.sender = SENDER_SYSTEMSTATEMACHINE;

                // Sending it back to the initial Sender
                sendToSender(initialSenderMove, returnedProtocol);

                // As a error occured we set the state back to idle for inSimulation
                inSimulationState = 0;
                return;
            }
            
            // Increase to next state for chessEngineMove
            inSimulationPlayerMoveState++;
        }

        // Otherwise go through shared inSimulation states
        else{
            inSimulationRobotMove(inSimulationPlayerMoveState);
        }
    }


    // StateMachine for simulation chess engine move
    void inSimulationStateMachineChessEngineMove(){

        if(inSimulationChessEngineMoveState == 6){
            //Create the protocol for sending back to the initial sender
            ibo1_irc_api::Protocol sendProtocolToInitialSender;
            sendProtocolToInitialSender.cmd = initialSenderMoveCmd;
            sendProtocolToInitialSender.sender = SENDER_SYSTEMSTATEMACHINE;
            sendProtocolToInitialSender.data = inSimulationDataStorage;


            sendToSender(initialSenderMove, sendProtocolToInitialSender);
            inSimulationState = 0;
        }

        // Waiting till we received return from ChessWrapper
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

        // Otherwise go through shared inSimulation states
        else{
            inSimulationRobotMove(inSimulationChessEngineMoveState);
        }
    }


    // StateMachine for simulation playing a game of Chess
    void inSimulationStateMachine(){

        // Checking if we are in going down the playerMove state
        if(inSimulationState == 2){
            inSimulationPlayerMove();
        }

        // Checking if we are in going down the chessEngineMove state
        else if(inSimulationState == 1){
            inSimulationStateMachineChessEngineMove();
        }

        // Checking if inSimulationState == 0 then we are in IDLE and waiting for chessEngine or playerMove
        else if(inSimulationState == 0){
            switch (returnedProtocol.cmd)
            {
                case CMD_INTERNAL_PLAYERMOVE:{
                    //Getting the initial sender

                    ibo1_irc_api::Protocol getPlayerMoveProtocol;
                    getPlayerMoveProtocol.cmd = returnedProtocol.cmd;
                    getPlayerMoveProtocol.data = returnedProtocol.data;
                    getPlayerMoveProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                    sendToSender(SENDER_CHESSWRAPPER, getPlayerMoveProtocol);

                    initialSenderMove = returnedProtocol.sender;
                    initialSenderMoveCmd = returnedProtocol.cmd;
                    inSimulationDataStorage = returnedProtocol.data;

                    inSimulationState = 2;
                    inSimulationPlayerMoveState = 0;
                    didCastle = false;
                    break;
                }

                case CMD_INTERNAL_CHESSENGINEMOVE:{

                    ibo1_irc_api::Protocol getChessEngineMoveProtocol;
                    getChessEngineMoveProtocol.cmd = returnedProtocol.cmd;
                    getChessEngineMoveProtocol.data = returnedProtocol.data;
                    getChessEngineMoveProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                    sendToSender(SENDER_CHESSWRAPPER, getChessEngineMoveProtocol);

                    // Increasing the inSimulation state to 1 as we need to go down the chessEngineMove state
                    inSimulationState = 1;
                    inSimulationChessEngineMoveState = 0;
                    didCastle = false;
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
    ros::init(argc, argv, "systemStateMachine");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber systemStateMachine_sub = nh.subscribe("/ircSystemStateMachine", 1, &systemStateMachineMessageReceived);

    ros::Publisher server_pub = nh.advertise<ibo1_irc_api::Protocol>("ircServer", 10);
    ros::Publisher chessWrapper_pub = nh.advertise<ibo1_irc_api::Protocol>("ircChessWrapper", 10);
    ros::Publisher createTarget_pub = nh.advertise<ibo1_irc_api::Protocol>("ircCreateTarget", 10);
    ros::Publisher robotArmStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("my_gen3/ircRobotArmStateMachine", 10);

    server_pub_ptr = &server_pub;
    chessWrapper_pub_ptr = &chessWrapper_pub;
    createTarget_pub_ptr = &createTarget_pub;
    robotArmStateMachine_pub_ptr = &robotArmStateMachine_pub;

    ros::Rate rate(10);


    while(ros::ok()){
        systemStateMachineLogic();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}