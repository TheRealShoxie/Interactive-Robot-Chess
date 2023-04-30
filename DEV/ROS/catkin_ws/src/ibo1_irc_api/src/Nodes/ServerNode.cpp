/*
 * ServerNode - Ros node which is used for ircServer
 * <p>
 * This file enables the external to internal system communication. It allows Systems from outside the
 * ros system to communicated to. This is done by using TCP communication which is described in the 
 * IRCServer.cpp. It forwards commands from external systems to the ircSystemStateMachineNode.
 * 
 * It uses the ServerNodeHelper.h to convert from external to internal protocol.
 * 
 * It reads in users from a list to use as login.
 * 
 * 
 * 
 * <p>
 * This Node provides following functionality:
 *  - Login
 * 
 * Outside from that functionality it forwards commands to the ircSystemStateMachine Node.
 * Described in SystemStateMachineNode.cpp
 * 
 * 
 * <p>
 * This file is launched with the ircSystem.launch file
 * 
 * <p>
 * It uses the paramServer to get usersFilePath for the txt file which saves the users information
 * It uses the paramServer to get the systemDebug value if debugging is enabled or disabled
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see IRCServer.h
 * @see CMakeLists.txt
 * @see ircSystem.launch
 * @see SystemStateMachineNode.cpp
 * @see ServerNodeHelper.h
 * @see NodeHelper.h
 * @see FileHandler.h
 * @see Users.h
 * @see User.h
 * @see internalProtocol.h
 * @see Protocol.h
*/


    // ////////// //
    // Includes.  //
    // ////////// //

#include <ibo1_irc_api/Server/IRCServerNodeHelper.h>
#include<ibo1_irc_api/Utility/NodeHelper.h>


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //

// For ros subscribers & publisher
ibo1_irc_api::Protocol returnedProtocol;
ros::Publisher* systemStateMachine_pub_ptr;

// For debug
bool systemDebug = false;

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

// Callback on /ircSystem/ircServer. 
void serverMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;

    //Only prints if systemDebug true
    ROS_INFO_COND(systemDebug, "I received from: %d", returnedProtocol.sender);
    ROS_INFO_COND(systemDebug, "I received the CMD: %d", returnedProtocol.cmd);

}




/*
---------------------------------------------------------------------------------------------------------------------------------
*/

// Main logic
void communicationLogic(int bufferSizeData, IRCServer &server, Users &users){
    vector<BYTE> receivedData;
    vector<BYTE> answer;

    ROS_INFO_COND(systemDebug, "Size of data: %d", bufferSizeData);

    // Checking if data size is greater 0 or 1000000 then get the data
    if(bufferSizeData > 0 && bufferSizeData < 100000) receivedData = server.getData(bufferSizeData);

    // Otherwise set bufferSizeData to -1
    else if(bufferSizeData < 0){
        ROS_WARN_COND(systemDebug, "Buffer Size of data was smaller 0 thus errorCode: %d", bufferSizeData);
    }

    // Otherwise if bufferSize 0 means no data was supplied
    else if(bufferSizeData > 100000){
        ROS_WARN_COND(systemDebug, "Buffer Size greater than allowed: %d", bufferSizeData);
    }

    // If our bufferSize Data is greater or equal 0 
    if(bufferSizeData >= 0){


        // Checking what the Command we got was
        switch (server.getClientCommand())
        {

            // Login
            case CMD_LOGIN: {
                
                // If debug then create the string from the data received and print it
                if(systemDebug){
                    string loginString = "";
                    DataCreator::convertBytesToString(receivedData, loginString);
                    ROS_INFO_COND(systemDebug, "Login user with name and password: %s", loginString.c_str());
                }
                
                // Log in the user
                IRCServerNodeHelper::cmdUserLogin(receivedData, server, users, answer);
                break;
            }

            // Create user
            case CMD_CREATEUSER:
                // Still needs to be implemented.
                ROS_WARN_COND(systemDebug, "Functionality not implemented in the system");
                break;
            

            // Getting the chess engine
            case CMD_GETCHESSENGINES:{

                ROS_INFO_COND(systemDebug, "Get Chess engine names");

                // Converting external to internal command
                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_GETCHESSENGINES);
                BYTE returnedCommand;

                // Forwarding command and waiting till we got a response back from the node we forwarded to
                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);

                ROS_INFO_COND(systemDebug, "Command to send back to external device: %d", returnedCommand);
                
                returnedProtocol.cmd = (BYTE)0x00;
                break;
            }
            
            // Starting a chess engine
            case CMD_STARTCHESSENGINE:{

                // If debug then create the string from the data received and print it
                if(systemDebug){
                    string chessEngine = "";
                    DataCreator::convertBytesToString(receivedData, chessEngine);
                    ROS_INFO_COND(systemDebug, "Starting Chess engine: %s", chessEngine.c_str());
                }

                // Converting external to internal command
                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_STARTCHESSENGINE);
                BYTE returnedCommand;

                // Forwarding command and waiting till we got a response back from the node we forwarded to
                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);


                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);

                ROS_INFO_COND(systemDebug, "Command to send back to external device: %d", returnedCommand);

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;
            }
            
            // Stop chess engine
            case CMD_STOPCHESSENGINE:{

                ROS_INFO_COND(systemDebug, "Stop chess engine");

                // Converting external to internal command
                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_STOPCHESSENGINE);
                BYTE returnedCommand;

                // Forwarding command and waiting till we got a response back from the node we forwarded to
                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);


                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);

                ROS_INFO_COND(systemDebug, "Command to send back to external device: %d", returnedCommand);

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;
            }

            // Player move
            case CMD_PLAYERMOVE:{
                // If debug then create the string from the data received and print it
                if(systemDebug){
                    string playerMove = "";
                    DataCreator::convertBytesToString(receivedData, playerMove);
                    ROS_INFO_COND(systemDebug, "Player Move: %s", playerMove.c_str());
                }

                // Converting external to internal command
                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_PLAYERMOVE);
                BYTE returnedCommand;

                // Forwarding command and waiting till we got a response back from the node we forwarded to
                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);


                ROS_INFO_COND(systemDebug, "Command to send back to external device: %d", returnedCommand);

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;

            }

            // Get Chess Engine Move
            case CMD_CHESSENGINEMOVE:{

                ROS_INFO_COND(systemDebug, "Get chess engine move");

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_CHESSENGINEMOVE);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);


                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);

                // If debug then create the string from the data received and print it
                if(systemDebug){
                    string chessEngineMove = "";
                    DataCreator::convertBytesToString(receivedData, chessEngineMove);
                    ROS_INFO_COND(systemDebug, "Chess Engine Move to send back to external device: %s", chessEngineMove.c_str());
                    ROS_INFO_COND(systemDebug, "Command to send back to external device: %d", returnedCommand);
                }

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;

            }

            // Set System into full sim
            case CMD_SYSTEMFULLSIM:{
                ROS_INFO_COND(systemDebug, "Set system into full sim");

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_SYSTEMFULLSIM);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);
                
                ROS_INFO_COND(systemDebug, "Command to send back to external device: %d", returnedCommand);

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;
            }

            case CMD_SYSTEMWITHOUTSIM:{
                ROS_INFO_COND(systemDebug, "Set system into no sim");

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_SYSTEMWITHOUTSIM);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);
                
                ROS_INFO_COND(systemDebug, "Command to send back to external device: %d", returnedCommand);

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;
            }

            default:
                break;
        }
    }

    

    // Sending back the answer
    server.sendAnswer(answer);

}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/


int main (int argc, char **argv){
    ros::init(argc, argv, "ircServer");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber server_sub = nh.subscribe("/ircSystem/ircServer", 1, &serverMessageReceived);

    ros::Publisher systemStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("ircSystemStateMachine", 10);

    systemStateMachine_pub_ptr = &systemStateMachine_pub;
    

    ros::Rate rate(10);

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
    string usersFilePathName = "";


    // Getting ChessEngineFilePath from param server with a default value if it doesn't exist
    ros::param::param<string>("/usersNameFilePath", usersFilePathName, 
                              "/home/omar/Uni/Major_Project/Interactive-Robot-Chess/DEV/ROS/catkin_ws/src/ibo1_irc_api/data/Users/users.txt");

    
    Users users;

    // Trying to create the Users from supplied filepath. if it is not possible print an error and return
    try{
        users = Users(usersFilePathName);
    } catch(invalid_argument e){
        ROS_ERROR("%s - Node will be shutdown", e.what());
        return -1;
    }

    // Creating an instance of server
    IRCServer server = IRCServer(54001);

    // Amount of retrying to start server
    int triesToStartServer = 0;
    bool serverStarted = false;

    // Tries to start the server, if successful continue, otherwise we retry max 4 times and close node with error
    while(!serverStarted){
        if(triesToStartServer <= 4){
            //Try to open connection again if it doesnt work retry
            try{
                server.initiateServerSocket();
                ROS_INFO_COND(systemDebug, "Client Socket: %d", server.getClientSocket());
                serverStarted = true;
            } catch(runtime_error e){
                ROS_WARN_COND(systemDebug, "Server wasn't started retrying");
                ROS_WARN_COND(systemDebug, "With following error: %s", e.what());
                triesToStartServer++;
                ros::Duration(2).sleep();
            }  catch(...){
                ROS_ERROR("UNEXPECTED ERROR WHEN TRYING TO START SERVER!");
            }
        }
        else{
            ROS_ERROR("Server couldnt be started with following message");
            return -1;
        }
    }
    
    

    int bufferSizeData = 0;

    while(ros::ok()){

        ROS_INFO_COND(systemDebug, "Waiting for message....");

        bufferSizeData = server.commandExtraction();

        // If we disconnected break out of loop
        if(bufferSizeData == -2){
            ROS_INFO_COND(systemDebug, "Was told to disconnect");
            break;
        }
        // If we received bytes not in the amount of protocol error occured print the error and do not exit
        else if(bufferSizeData == -4){
            ROS_ERROR_COND(systemDebug, "Bytes rescv is not length of proctocol(5)!");
        }

        // Getting into communication logic
        communicationLogic(bufferSizeData, server, users);

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}