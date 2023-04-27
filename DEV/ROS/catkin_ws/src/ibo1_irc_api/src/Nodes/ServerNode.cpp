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

        cout << "I get here!" << endl;
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
                BYTE returnedCommand;

                cout << "I get before here!" << endl;
                NodeHelper::forwarder(returnedProtocol, receivedData, CMD_INTERNAL_GETCHESSENGINES, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                cout << "I get here!" << endl;

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
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, CMD_INTERNAL_STARTCHESSENGINE, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);


            
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
                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, CMD_INTERNAL_STOPCHESSENGINE, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);


            
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

                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, CMD_INTERNAL_PLAYERMOVE, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

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

                BYTE returnedCommand;

                NodeHelper::forwarder(returnedProtocol, receivedData, CMD_INTERNAL_CHESSENGINEMOVE, SENDER_SERVER, SENDER_SYSTEMSTATEMACHINE, answer, returnedCommand, systemStateMachine_pub_ptr);

                cout << "Data to return: ";
                for(auto &b : answer){
                    cout << (char)(int(b));
                }
                cout << endl;
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

    // while(server_pub.getNumSubscribers() == 0){
    //     rate.sleep();
    // }


    Users users(usersFilePathName);

    IRCServer server = IRCServer(54001);
    server.initiateServerSocket();
    cout << "Client Socket: " << server.getClientSocket() << endl;

    int bufferSizeData = 0;

    while(ros::ok()){

        cout << "Waiting for message!" << endl;

        bufferSizeData = server.commandExtraction();

        // If we disconnected break out of loop
        if(bufferSizeData == -2){
            cout << "Was told to disconnect!" << endl;
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