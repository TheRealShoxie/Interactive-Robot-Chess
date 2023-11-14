/*
 * ChessWrapperNode - Ros node which is used for ircChess
 * <p>
 * This file describes the internal communication and execution of its respective commands.
 * It communicates with the ircSystemState machine to receive and respond with commands.
 * For how it is connected to the internal system refer to SystemStateMachineNode.cpp
 * 
 * The definition of the internal protocol is defined in the folder data/Protocol/InternalProtocol.h
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
 * 
 * <p>
 * This file is launched with the ircSystem.launch file
 * 
 * <p>
 * It uses the paramServer to get chessEnginesFilepath for the txt file which saves the chessEngines
 * It uses the paramServer to get the systemDebug value if debugging is enabled or disabled
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see ChessEngine.h
 * @see CMakeLists.txt
 * @see ircSystem.launch
 * @see SystemStateMachineNode.cpp
 * @see InternalProtocol.h
 * @see DataCreator.h
 * @see FileHander.h
*/


    // ////////// //
    // Includes.  //
    // ////////// //

// Ros include
#include <ros/ros.h>

// Internal includes
#include <ibo1_irc_api/Utility/FileHandler.h>
#include <ibo1_irc_api/Chess/ChessEngine.h>

// Created messsages includes
#include <ibo1_irc_api/Protocol.h>

/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //
// For ros subscribers & publisher
ibo1_irc_api::Protocol returnedProtocol;
ros::Publisher* systemStateMachine_pub_ptr;

// For ChessEngine
ChessEngine *chessEnginePointer;

// For debug
bool systemDebug = false;

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

// Callback function for subscriber on /ircSystem/ircChessWrapper
void chessWrapperMessageReceived(const ibo1_irc_api::Protocol& msg){
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
        // Sending to systemStateMachine
        case SENDER_SYSTEMSTATEMACHINE:{
            systemStateMachine_pub_ptr->publish(sendProtocol);
            break;
        }
        
        // Default do nothing
        default:
            break;
    }
}


// Main Logic for ChessWrapper
void wrapperLogic(vector<ChessEngineDefinitionStruct>& chessEngines){

    // Creating temporary variables
    vector<BYTE> response;
    ibo1_irc_api::Protocol sendProtocol;

    BYTE gotCMD = returnedProtocol.cmd;


    // Checking which command we have
    switch(gotCMD){


        //Get chess engine names
        case CMD_INTERNAL_GETCHESSENGINES:{
            
            ROS_INFO_COND(systemDebug, "I got request to send back possible chess engines");

            // Getting the chessEngine names from chessEngine to send back as data
            string chessEngineNames = "";
            for(ChessEngineDefinitionStruct ce : chessEngines){
                chessEngineNames += ce.name;
                chessEngineNames += "\u241f";
            }

            // Converting the string data into byte byte data
            copy(chessEngineNames.begin(), chessEngineNames.end(), std::back_inserter(response));

            //Creating the protocol to be send back
            sendProtocol.cmd = CMD_INTERNAL_GETCHESSENGINES;
            sendProtocol.sender = SENDER_CHESSWRAPPER;
            sendProtocol.data = response;
            sendToSender(returnedProtocol.sender, sendProtocol);

            break;
        }

        // Start chess engine
        case CMD_INTERNAL_STARTCHESSENGINE:{
            // Setting up temp variables and converting Bytes to string
            string toStartChessEngineName = "";
            string toStartChessEngineFilePathName;
            DataCreator::convertBytesToString(returnedProtocol.data, toStartChessEngineName);

            ROS_INFO_COND(systemDebug, "ChessEngine Name to start %s", toStartChessEngineName.c_str());

            // Looking through the file to find the chess engine
            for(ChessEngineDefinitionStruct ce : chessEngines){
                if(ce.name.compare(toStartChessEngineName) == 0){
                    toStartChessEngineFilePathName = ce.filePathName;
                }
            }

            ROS_INFO_COND(systemDebug, "Filepath of the chessengine to start: %s", toStartChessEngineFilePathName.c_str());

            //If filepath is empty we couldn't find that chess engine send back error 
            if(toStartChessEngineFilePathName.empty()){
                sendProtocol.cmd = ERROR_INTERNAL_CMD_CHESSENGINEDOESNTEXIST;
                sendProtocol.data = response;
                ROS_WARN_COND(systemDebug, "Couldnt find Chess Engine Name!");
            }
            // Otherwise we have found the chess engine
            else{

                // Checking if a chessEngine already exist or is running. If yes close it and delete it
                if(!(chessEnginePointer == NULL)){
                    delete chessEnginePointer;
                }

                // Start the chess engine
                chessEnginePointer = new ChessEngine(toStartChessEngineFilePathName);
                sendProtocol.cmd = CMD_INTERNAL_STARTCHESSENGINE;
                ROS_INFO_COND(systemDebug, "Started chess engine");
            }

            //Prepare protocol to be send back and send it back to where request came from
            sendProtocol.sender = SENDER_CHESSWRAPPER;
            sendProtocol.data = response;
            sendToSender(returnedProtocol.sender, sendProtocol);
            break;
        }

        // Stop Chess engine
        case CMD_INTERNAL_STOPCHESSENGINE:{

            // Check if a chess engine is not running, then send back an error that none was running
            if(chessEnginePointer == NULL){
                sendProtocol.cmd = ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING;
                ROS_WARN_COND(systemDebug, "No chess engine running");
            }
            
            // Otherwise close and stop the chess engine
            else{
                delete chessEnginePointer;
                chessEnginePointer = NULL;
                sendProtocol.cmd = CMD_INTERNAL_STOPCHESSENGINE;
                ROS_INFO_COND(systemDebug, "Stopped chess engine");
            }

            //Prepare protocol to be send back and send it back to where request came from
            sendProtocol.sender = SENDER_CHESSWRAPPER;
            sendProtocol.data = response;
            sendToSender(returnedProtocol.sender, sendProtocol);
            break;
        }


        // Make a player move
        case CMD_INTERNAL_PLAYERMOVE:{

            // Check if a chess engine is not running, then send back an error that none was running
            if(chessEnginePointer == NULL){
                sendProtocol.cmd = ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING;
                ROS_WARN_COND(systemDebug, "No chess engine running");
            }
            
            // Otherwise try to make player move
            else{
                string moveCommand = "";
                DataCreator::convertBytesToString(returnedProtocol.data, moveCommand);

                ROS_INFO_COND(systemDebug, "Player Move send: %s", moveCommand.c_str());
                ROS_INFO_COND(systemDebug, "Current Board state: %s", chessEnginePointer->getChessBoardString().c_str());

                // Trying to make player move. toReturnProtocolCmd can be an error if illegal move
                BYTE toReturnProtocolCmd;
                chessEnginePointer->playerMove(toReturnProtocolCmd, moveCommand);

                ROS_INFO_COND(systemDebug, "Current Board state after player move: %s", chessEnginePointer->getChessBoardString().c_str());

                //Prepare protocol to be send back and send it back to where request came from
                sendProtocol.cmd = toReturnProtocolCmd;
                sendProtocol.sender = SENDER_CHESSWRAPPER;
                sendProtocol.data = response;
                sendToSender(returnedProtocol.sender, sendProtocol);
                break;
            }
        }

        // Get chess engine move
        case CMD_INTERNAL_CHESSENGINEMOVE:{

            // Check if a chess engine is not running, then send back an error that none was running
            if(chessEnginePointer == NULL){
                sendProtocol.cmd = ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING;
                ROS_WARN_COND(systemDebug, "No chess engine running");
            }
            
            // Otherwise get the chessEngineMove
            else{
                string chessEngineMove = "";
                BYTE toReturnProtocolCmd;

                // Get chessengine move
                chessEnginePointer->chessEngineMove(toReturnProtocolCmd, chessEngineMove);

                // Converting string to bytes for protocol
                copy(chessEngineMove.begin(), chessEngineMove.end(), std::back_inserter(response));

                ROS_INFO_COND(systemDebug, "Chess engine move: %s", chessEngineMove.c_str());
                ROS_INFO_COND(systemDebug, "Current Board state: %s", chessEnginePointer->getChessBoardString().c_str());

                //Prepare protocol to be send back and send it back to where request came from
                sendProtocol.cmd = toReturnProtocolCmd;
                sendProtocol.sender = SENDER_CHESSWRAPPER;
                sendProtocol.data = response;
                sendToSender(returnedProtocol.sender, sendProtocol);
                break;
            }
        }

        // Check if last move was a castle move
        case CMD_INTERNAL_LASTMOVECASTLEMOVE:{

            // Check if a chess engine is not running, then send back an error that none was running
            if(chessEnginePointer == NULL){
                sendProtocol.cmd = ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING;
                ROS_WARN_COND(systemDebug, "No chess engine running");
            }

            // Otherwise check if last move was castle move
            else{
                
                // Getting info if last move was castle move
                bool lastMoveCastleMove = chessEnginePointer->wasLastMoveCastleMove();


                BYTE toReturnProtocolCmd;

                // Checking if last move was castle move then send back same cmd
                if(lastMoveCastleMove){
                    toReturnProtocolCmd = CMD_INTERNAL_LASTMOVECASTLEMOVE;
                    ROS_INFO_COND(systemDebug, "Last move was a castle move");
                }

                // Otherwise send back last move was not castle move
                else{
                    toReturnProtocolCmd = ERROR_INTERNAL_CMD_LASTMOVEWASNOTCASTLEMOVE;
                    ROS_INFO_COND(systemDebug, "Last move was not a castle move");
                }

                //Prepare protocol to be send back and send it back to where request came from
                sendProtocol.cmd = toReturnProtocolCmd;
                sendProtocol.sender = SENDER_CHESSWRAPPER;
                sendToSender(returnedProtocol.sender, sendProtocol);
            }
        }


        default:
            break;
    }

    // Resseting returnedProtocol cmd 
    returnedProtocol.cmd = (BYTE)0x00; 

}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////// //
    // Main.  //
    // ////// //


// Main function that gets started by ircSystem.launch file
int main (int argc, char **argv){

    // ROS node and spinner setup
    ros::init(argc, argv, "chessWrapper");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    // Subscriber and publisher setup
    ros::Subscriber server_sub = nh.subscribe("/ircSystem/ircChessWrapper", 1, &chessWrapperMessageReceived);

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


    // FilePath for where the chessEngine Names and their paths are defined
    string chessEnginesFilePathName = "";


    // Getting ChessEngineFilePath from param server with a default value if it doesnt exist
    ros::param::param<string>("/chessEngineNameFilePath", chessEnginesFilePathName, 
                              "/home/omar/Uni/Major_Project/Interactive-Robot-Chess/DEV/ROS/catkin_ws/src/ibo1_irc_api/data/Chess/chessEngines.txt");



    vector<ChessEngineDefinitionStruct> chessEngines;

    // Reading the file and setting the possible chessEngines. If it cannot be read throw error and close program
    try{
        chessEngines = FileHandler::readChessEngines(chessEnginesFilePathName);
    } catch(invalid_argument e){
        ROS_ERROR("%s - Node will be shutdown", e.what());
        return -1;
    }
    

    ros::Rate rate(10);

    // Main while loop till ros is closed then we exit
    while(ros::ok()){
        
        // Main logic keeps getting repeatedly called while node is running
        wrapperLogic(chessEngines);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}