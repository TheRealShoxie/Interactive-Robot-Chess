#include <ibo1_irc_api/Server/IRCServerNodeHelper.h>
#include<ibo1_irc_api/Utility/NodeHelper.h>


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //
static string usersFilePathName = "/home/omar/Uni/Major_Project/Interactive-Robot-Chess/DEV/ROS/catkin_ws/src/ibo1_irc_api/data/Users/users.txt";

// For ros subscribers & publisher
ibo1_irc_api::Protocol returnedProtocol;
ros::Publisher* systemStateMachine_pub_ptr;

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //
void serverMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    cout << "I received following on server: " << endl;
    cout << "I received from: " << (int)returnedProtocol.sender << endl;
    cout << "CmdByte: " << (int)returnedProtocol.cmd << endl;

}




/*
---------------------------------------------------------------------------------------------------------------------------------
*/

void communicationLogic(int bufferSizeData, IRCServer &server, Users &users){
    vector<BYTE> receivedData;
    vector<BYTE> answer;

    if(bufferSizeData > 0 && bufferSizeData < 100000) receivedData = server.getData(bufferSizeData);
    
    cout << "CLIENT COMMAND is:" << (int)(server.getClientCommand()) << endl;

    if(bufferSizeData >= 0){

        switch (server.getClientCommand())
        {
            case CMD_LOGIN: {
                IRCServerNodeHelper::cmdUserLogin(receivedData, server, users, answer);
                break;
            }

            case CMD_CREATEUSER:
                // Still needs to be implemented.
                break;
            

            case CMD_GETCHESSENGINES:{
                cout << "-------Get ChessEngine names-------" << endl;
                cout << "Data received: ";
                for(auto &b : receivedData){
                    cout << (char)(int(b));
                }
                cout << endl;

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_GETCHESSENGINES);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);
                cout << "Command to send back: " << (int)returnedCommand << endl;
                cout << "-------------------------";
                
                returnedProtocol.cmd = (BYTE)0x00;
                break;
            }
            
            case CMD_STARTCHESSENGINE:{
                cout << "-------Start ChessEngine-------" << endl;
                cout << "Data received: ";
                for(auto &b : receivedData){
                    cout << (char)(int(b));
                }
                cout << endl;

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_STARTCHESSENGINE);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);


                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);
                cout << "Command to send back: " << (int)returnedCommand << endl;
                cout << "-------------------------";

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;
            }
            
            case CMD_STOPCHESSENGINE:{
                cout << "-------Stop ChessEngine-------" << endl;
                cout << "Data received: ";
                for(auto &b : receivedData){
                    cout << (char)(int(b));
                }
                cout << endl;

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_STOPCHESSENGINE);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);


                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);
                cout << "Command to send back: " << (int)returnedCommand << endl;
                cout << "-------------------------";

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;
            }

            case CMD_PLAYERMOVE:{
                cout << "-------Player Move-------" << endl;
                cout << "Data received: ";
                for(auto &b : receivedData){
                    cout << (char)(int(b));
                }
                cout << endl;

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_PLAYERMOVE);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);
                cout << "Command to send back: " << (int)returnedCommand << endl;
                cout << "-------------------------";

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;

            }

            case CMD_CHESSENGINEMOVE:{
                cout << "-------Chess Engine Move-------" << endl;
                cout << "Data received: ";
                for(auto &b : receivedData){
                    cout << (char)(int(b));
                }
                cout << endl;

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_CHESSENGINEMOVE);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                cout << "Data to return: ";
                for(auto &b : answer){
                    cout << (char)(int(b));
                }
                cout << endl;

                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);
                cout << "Command to send back: " << (int)returnedCommand << endl;
                cout << "-------------------------";

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;

            }

            case CMD_SYSTEMFULLSIM:{
                cout << "-------SYSTEM set full sim-------" << endl;

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_SYSTEMFULLSIM);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);
                cout << "Command to send back: " << (int)returnedCommand << endl;
                cout << "-------------------------";

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;
            }

            case CMD_SYSTEMWITHOUTSIM:{
                cout << "-------SYSTEM set without sim-------" << endl;

                BYTE sendCommand = IRCServerNodeHelper::externalToInternalCmdConverter(CMD_SYSTEMWITHOUTSIM);
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, sendCommand, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                returnedCommand = IRCServerNodeHelper::internalToExternalCmdConverter(returnedCommand);
                cout << "Command to send back: " << (int)returnedCommand << endl;
                cout << "-------------------------";

                returnedProtocol.cmd = (BYTE)0x00;
                server.setClientCommand(returnedCommand);
                break;
            }

            default:
                break;
        }
    }

    


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

    ros::Subscriber server_sub = nh.subscribe("/ircServer", 1, &serverMessageReceived);

    ros::Publisher systemStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("ircSystemStateMachine", 10);

    systemStateMachine_pub_ptr = &systemStateMachine_pub;
    

    ros::Rate rate(10);


    Users users(usersFilePathName);

    
    

    IRCServer server = IRCServer(54001);

    
    // Amount of retrying to start server
    int triesToStartServer = 0;
    bool serverStarted = false;

    while(!serverStarted){
        if(triesToStartServer <= 4){
            //Try to open connection again if it doesnt work retry
            try{
                server.initiateServerSocket();
                cout << "Client Socket: " << server.getClientSocket() << endl;
                serverStarted = true;
            } catch(...){
                ROS_WARN("Server wasn't started retrying");
                triesToStartServer++;
                ros::Duration(2).sleep();
            }  
        }
        else{
            ROS_ERROR("Server couldnt be started");
            return -1;
        }
    }
    
    

    int bufferSizeData = 0;

    while(ros::ok()){

        cout << "Waiting for message!" << endl;

        bufferSizeData = server.commandExtraction();

        // If we disconnected break out of loop
        if(bufferSizeData == -2){
            cout << "Was told to disconnect!" << endl;
            break;
        }
        else if(bufferSizeData == -4){
            ROS_ERROR("Bytes rescv is not length of proctocol(5)!");
            ROS_ERROR("Closing socket!");
            server.closeClientSocket();
            break;
        }

        cout << "Size of to come Data: " << bufferSizeData << endl;
        communicationLogic(bufferSizeData, server, users);

        cout << "\n--------------------------------------------------------" << endl;

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}