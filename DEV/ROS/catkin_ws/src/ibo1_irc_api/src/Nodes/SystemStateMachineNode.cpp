/*
 * SystemStateMachineNode - Ros node which is used for ircSystemStateMachine
 * <p>
 * This Node connects the whole system together. It has multiple different states.
 * The system can be set into a full robot simulation, partial simulation, no simulation, and real with robot.
 * 
 * Currently only full simulation and no simulation is implemented.
 * 
 * It connects on how a playerMove and chessMove are done by their retrospective state machines.
 * 
 * In the idle state the system functions as a execture and forwarder.
 * If a chess engine gets started it goes into its corresponding simulation, no simulation and real robot states.
 * There it defines how player moves and chess moves are to be processed.
 * 
 * It can exit that system state and returning into idle by getting a chess engine stop move.
 * 
 * 
 * This file describes the internal communication and execution of its respective commands.
 * It communicates with the ircSystemState machine to receive and respond with commands.
 * For how it is connected to the internal system refer to SystemStateMachineNode.cpp
 * 
 * The definition of the internal protocol is defined in the folder data/Protocol/InternalProtocol.h
 * 
 * 
 * <p>
 * 
 * 
 * <p>
 * This Node provides following functionality:
 * 
 *  - Get possible chess engines
 *  - Start a chess engine
 *  - Stop a chess engine
 *  - Player move
 *  - Chess engine move
 *  - last move castle move
 *  - Robot arm move
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
 * @see ChessEngine.h
 * @see CMakeLists.txt
 * @see ircSystem.launch
 * @see ChessWrapperNode.cpp
 * @see CreateTargetNode.cpp
 * @see RobotArmStateMachineNode.cpp
 * @see SystemStateMachineHelper.h
 * @see ServerNode.cpp
 * @see DataCreator.h
 * @see DataChecker.h
 * @see Protocol.h
 * @see InternalProtocol.h
*/


    // ////////// //
    // Includes.  //
    // ////////// //

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
bool isNoSimulation = true;
bool isSimulationFull = false;
bool isPartialSimulation = false;
bool isReal = false;


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

// For debug
bool systemDebug = false;


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

// Callback function for subscriber on /ircSystem/ircSystemStateMachine
void systemStateMachineMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    //Only prints if systemDebug true
    ROS_INFO_COND(systemDebug, "I received from: %d", returnedProtocol.sender);
    ROS_INFO_COND(systemDebug, "I received the CMD: %d", returnedProtocol.cmd);
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
    bool secondMoveOfCastleMove(){
        
        string moveCmd = "";
        string newMoveCmd = "";
        DataCreator::convertBytesToString(inSimulationDataStorage, moveCmd);


        //White Castle King side
        if(moveCmd.compare("e1g1") == 0){
            newMoveCmd = "h1f1";
        }
        //White Castle Queen side
        else if(moveCmd.compare("e1c1") == 0){
            newMoveCmd = "a1d1";
        }
        //Black Castle King side
        else if(moveCmd.compare("e8g8") == 0){
            newMoveCmd = "h8f8";
        }
        //Black Castle Queen side
        else if(moveCmd.compare("e8c8") == 0){
            newMoveCmd = "a8d8";
        }
        else{
            return false;
        }

        
        didCastle = true;
        inSimulationDataStorageForCastling = inSimulationDataStorage;
        inSimulationDataStorage.clear();
        copy(newMoveCmd.begin(), newMoveCmd.end(), std::back_inserter(inSimulationDataStorage));
        return true;

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
                ROS_INFO_COND(systemDebug, "inSimulationRobotMoveState: 6");
                // As we got back the not castle move we increase the inSimulationRobotMoveState
                inSimulationRobotMoveState++;
            }else{
                // We got a last move castle move true thus we update the command 
                if(secondMoveOfCastleMove()) inSimulationRobotMoveState = 1;
                else inSimulationRobotMoveState++;
            }

            
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

            ROS_INFO_COND(systemDebug, "inSimulationRobotMoveState: 5");
            ROS_INFO_COND(systemDebug, "inSimulationRobotMoveState: 5 - Did we already castle: %s", didCastle);

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

            ROS_INFO_COND(systemDebug, "inSimulationRobotMoveState: 4");

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

            ROS_INFO_COND(systemDebug, "inSimulationRobotMoveState: 3");

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
            
            ROS_INFO_COND(systemDebug, "inSimulationRobotMoveState: 1");

            //Create the protocol for setting the targets
            ibo1_irc_api::Protocol sendCreateTargetProtocol;
            sendCreateTargetProtocol.cmd = CMD_INTERNAL_SETTARGET;
            sendCreateTargetProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
            string test = "";
            DataCreator::convertBytesToString(inSimulationDataStorage, test);
            sendCreateTargetProtocol.data = inSimulationDataStorage;

            //Sending setTarget command to Create target
            sendToSender(SENDER_CREATETARGET, sendCreateTargetProtocol);

            ROS_INFO_COND(systemDebug, "inSimulationRobotMoveState: 2");
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

            ROS_INFO_COND(systemDebug, "inSimulationPlayerMoveState: 6");


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
            ROS_INFO_COND(systemDebug, "inSimulationChessEngineMoveState: 6");
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
    void inSimulationFullStateMachine(){

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

                    ROS_INFO_COND(systemDebug, "systemState: 1 - Inside FullSim State - Player move");
                    if(systemDebug){
                        string chessEngineMove = "";
                        DataCreator::convertBytesToString(returnedProtocol.data, chessEngineMove);
                        ROS_INFO_COND(systemDebug, "Player Move: %s", chessEngineMove.c_str());
                    }

                    // Getting protocol to send to chess wrapper
                    ibo1_irc_api::Protocol getPlayerMoveProtocol;
                    getPlayerMoveProtocol.cmd = returnedProtocol.cmd;
                    getPlayerMoveProtocol.data = returnedProtocol.data;
                    getPlayerMoveProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                    sendToSender(SENDER_CHESSWRAPPER, getPlayerMoveProtocol);


                    //Storing initial data received
                    initialSenderMove = returnedProtocol.sender;
                    initialSenderMoveCmd = returnedProtocol.cmd;
                    inSimulationDataStorage = returnedProtocol.data;

                    ROS_INFO_COND(systemDebug, "inSimulationState: 2");
                    ROS_INFO_COND(systemDebug, "inSimulationPlayerMoveState: 0");

                    inSimulationState = 2;
                    inSimulationPlayerMoveState = 0;
                    didCastle = false;
                    break;
                }

                case CMD_INTERNAL_CHESSENGINEMOVE:{

                    ROS_INFO_COND(systemDebug, "systemState: 1 - Inside FullSim State - Stop Chess Engine");

                    // Getting protocol to send to chess wrapper
                    ibo1_irc_api::Protocol getChessEngineMoveProtocol;
                    getChessEngineMoveProtocol.cmd = returnedProtocol.cmd;
                    getChessEngineMoveProtocol.data = returnedProtocol.data;
                    getChessEngineMoveProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                    sendToSender(SENDER_CHESSWRAPPER, getChessEngineMoveProtocol);

                    ROS_INFO_COND(systemDebug, "inSimulationState: 1");
                    ROS_INFO_COND(systemDebug, "inSimulationChessEngineMoveState: 0");

                    //Storing initial data received
                    initialSenderMove = returnedProtocol.sender;
                    initialSenderMoveCmd = returnedProtocol.cmd;

                    // Increasing the inSimulation state to 1 as we need to go down the chessEngineMove state
                    inSimulationState = 1;
                    inSimulationChessEngineMoveState = 0;
                    didCastle = false;

                    break;
                }

                case CMD_INTERNAL_STOPCHESSENGINE:{

                    ROS_INFO_COND(systemDebug, "systemState: 1 - Inside FullSim State - Stop Chess Engine");

                    //Getting the initial sender
                    BYTE initialSender = returnedProtocol.sender;

                    sendToSender(initialSender, SystemStateMachineHelper::systemStateMachineChessWrapperForwarder(returnedProtocol, chessWrapper_pub_ptr));

                    // After stopping the chessEngine we go back to initial idle state
                    systemState = 0;
                    break;
                }

                case ((BYTE)0x00):{
                    break;
                }

                default:{
                    // Getting the initial sender
                    BYTE initialSender = returnedProtocol.sender;

                    // Sending back to sender unrecognizable cmd
                    ibo1_irc_api::Protocol sendUnrecognizableCmdProtocol;
                    sendUnrecognizableCmdProtocol.cmd = ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE;
                    sendUnrecognizableCmdProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                    sendToSender(initialSender, sendUnrecognizableCmdProtocol);

                    break;
                }
            }
        }
    }

    // Playing chess without robot arms
    void inNoRobotArmState(){
        BYTE gotCMD = returnedProtocol.cmd;

        switch(gotCMD){
            case CMD_INTERNAL_PLAYERMOVE:{
                //Getting the initial sender
                BYTE initialSender = returnedProtocol.sender;

                ROS_INFO_COND(systemDebug, "systemState: 1 - Inside noRobotArm State - Player Move");

                if(systemDebug){
                    string chessEngineMove = "";
                    DataCreator::convertBytesToString(returnedProtocol.data, chessEngineMove);
                    ROS_INFO_COND(systemDebug, "Player Move: %s", chessEngineMove.c_str());
                }

                ibo1_irc_api::Protocol testProtocol = SystemStateMachineHelper::systemStateMachineChessWrapperForwarder(returnedProtocol, chessWrapper_pub_ptr);

                sendToSender(initialSender, testProtocol);
                break;
            }

            case CMD_INTERNAL_CHESSENGINEMOVE:{
                //Getting the initial sender
                BYTE initialSender = returnedProtocol.sender;

                ROS_INFO_COND(systemDebug, "systemState: 1 - Inside noRobotArm State - Chess Engine Move");

                ibo1_irc_api::Protocol testProtocol = SystemStateMachineHelper::systemStateMachineChessWrapperForwarder(returnedProtocol, chessWrapper_pub_ptr);

                if(systemDebug){
                    string chessEngineMove = "";
                    DataCreator::convertBytesToString(returnedProtocol.data, chessEngineMove);
                    ROS_INFO_COND(systemDebug, "Player Move: %s", chessEngineMove.c_str());
                }

                sendToSender(initialSender, testProtocol);
                break;
            }

            case CMD_INTERNAL_STOPCHESSENGINE:{
                //Getting the initial sender
                BYTE initialSender = returnedProtocol.sender;

                ibo1_irc_api::Protocol testProtocol = SystemStateMachineHelper::systemStateMachineChessWrapperForwarder(returnedProtocol, chessWrapper_pub_ptr);

                ROS_INFO_COND(systemDebug, "systemState: 1 - Inside noRobotArm State - Stopping Chess Engine");

            

                sendToSender(initialSender, testProtocol);
                
                // After stopping the chessEngine we go back to initial idle state
                systemState = 0;
                break;
            }

            case ((BYTE)0x00):{
                break;
            }

            default:{
                // Getting the initial sender
                BYTE initialSender = returnedProtocol.sender;

                // Sending back to sender unrecognizable cmd
                ibo1_irc_api::Protocol sendUnrecognizableCmdProtocol;
                sendUnrecognizableCmdProtocol.cmd = ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE;
                sendUnrecognizableCmdProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                sendToSender(initialSender, sendUnrecognizableCmdProtocol);

                break;
            }
        }
    }

    // For IDLE state system is a messageForwarder
    void idleState(){
        BYTE gotCMD = returnedProtocol.cmd;

        switch (gotCMD)
        {
            case CMD_INTERNAL_GETCHESSENGINES:{
                ROS_INFO_COND(systemDebug, "systemState: 0 - Forwarding getChessEngine Move");

                //Getting the initial sender
                BYTE initialSender = returnedProtocol.sender;

                sendToSender(initialSender, SystemStateMachineHelper::systemStateMachineChessWrapperForwarder(returnedProtocol, chessWrapper_pub_ptr));
                break;
            }

            case CMD_INTERNAL_STARTCHESSENGINE:{
                ROS_INFO_COND(systemDebug, "systemState: 0 - Forwarding startChessEngine");
                //Getting the initial sender
                BYTE initialSender = returnedProtocol.sender;

                sendToSender(initialSender, SystemStateMachineHelper::systemStateMachineChessWrapperForwarder(returnedProtocol, chessWrapper_pub_ptr));

                ROS_INFO_COND(systemDebug, "systemState: 1");
                //Moving to next state
                systemState = 1;
                break;
            }

            case CMD_INTERNAL_SYSTEMWITHOUTSIM:{
                isNoSimulation = true;
                isReal = false;
                isPartialSimulation = false;
                isSimulationFull = false;

                ROS_INFO_COND(systemDebug, "Set system without sim");


                // Sending back to sender unrecognizable cmd
                ibo1_irc_api::Protocol sendSystemWithoutSimProtocol;
                sendSystemWithoutSimProtocol.cmd = CMD_INTERNAL_SYSTEMWITHOUTSIM;
                sendSystemWithoutSimProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                sendToSender(returnedProtocol.sender, sendSystemWithoutSimProtocol);
                break;
            }

            case CMD_INTERNAL_SYSTEMFULLSIM:{
                isNoSimulation = false;
                isReal = false;
                isPartialSimulation = false;
                isSimulationFull = true;

                ROS_INFO_COND(systemDebug, "Set system with full sim");

                // Sending back to sender unrecognizable cmd
                ibo1_irc_api::Protocol sendSystemFullSimProtocol;
                sendSystemFullSimProtocol.cmd = CMD_INTERNAL_SYSTEMFULLSIM;
                sendSystemFullSimProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                sendToSender(returnedProtocol.sender, sendSystemFullSimProtocol);
                break;
            }

            case ((BYTE)0x00):{
                break;
            }
        
            default:{

                // Getting the initial sender
                BYTE initialSender = returnedProtocol.sender;

                // Sending back to sender unrecognizable cmd
                ibo1_irc_api::Protocol sendUnrecognizableCmdProtocol;
                sendUnrecognizableCmdProtocol.cmd = ERROR_INTERNAL_CMD_UNRECOGNIZABLE;
                sendUnrecognizableCmdProtocol.sender = SENDER_SYSTEMSTATEMACHINE;
                sendToSender(initialSender, sendUnrecognizableCmdProtocol);

                break;
            }

                
        }
    }


    // Main State machine
    void systemStateMachineLogic(){

        // Looking through all system states
        if(systemState == 1){
            
            // Checking if we are running no robot simulation
            if(isNoSimulation){
                inNoRobotArmState();
            }
            // Checking if we need to execute full simulation
            else if(isSimulationFull){
                inSimulationFullStateMachine();
            }
            // Otherwise checking if we are in real Simulation 
            else if(isReal){
                ROS_WARN("Not yet implemented");
            }
            // Simulation for chess engine and player move done not by robot arm
            else if(isPartialSimulation){
                ROS_WARN("Not yet implemented");
            }
            // None set error
            else{
                ROS_ERROR("No System state defined for simulation/real/partial/noSimulation");
            }
        }
        else if(systemState == 0){
            idleState();
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

    // Setting up ros subscribers and publishers
    ros::Subscriber systemStateMachine_sub = nh.subscribe("/ircSystem/ircSystemStateMachine", 1, &systemStateMachineMessageReceived);

    ros::Publisher server_pub = nh.advertise<ibo1_irc_api::Protocol>("ircServer", 10);
    ros::Publisher chessWrapper_pub = nh.advertise<ibo1_irc_api::Protocol>("ircChessWrapper", 10);
    ros::Publisher createTarget_pub = nh.advertise<ibo1_irc_api::Protocol>("ircCreateTarget", 10);
    ros::Publisher robotArmStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("/my_gen3/ircRobotArmStateMachine", 10);

    server_pub_ptr = &server_pub;
    chessWrapper_pub_ptr = &chessWrapper_pub;
    createTarget_pub_ptr = &createTarget_pub;
    robotArmStateMachine_pub_ptr = &robotArmStateMachine_pub;

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

    ros::Rate rate(10);


    while(ros::ok()){
        systemStateMachineLogic();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}