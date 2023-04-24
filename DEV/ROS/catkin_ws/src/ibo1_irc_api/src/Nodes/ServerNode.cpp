#include <ibo1_irc_api/Server/IRCServerNodeHelper.h>

#include <std_msgs/String.h>


    // ////////// //
    // Constants. //
    // ////////// //
static string usersFilePathName = "/home/omar/Uni/Major_Project/Interactive-Robot-Chess/DEV/ROS/catkin_ws/src/ibo1_irc_api/data/Users/users.txt";
ibo1_irc_api::Protocol returnedProtocol;

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //
void chessEngineMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    cout << "I received following on server from chessWrapper: " << endl;
    cout << "CmdByte: " << (int)returnedProtocol.cmd << endl;

}



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

void communicationLogic(int bufferSizeData, IRCServer &server, Users &users, ros::Publisher &server_pub){
    vector<BYTE> receivedData;
    vector<BYTE> answer;

    if(bufferSizeData > 0 && bufferSizeData < 100000) receivedData = server.getData(bufferSizeData);
    
    cout << "CLIENT COMMAND is:" << (int)(server.getClientCommand()) << endl;

    if(bufferSizeData >= 0){
        switch (server.getClientCommand())
        {
            case CMD_LOGIN: {
                IRCServerNodeHelper::cmdUserLogin(receivedData, server, users, answer);
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

                IRCServerNodeHelper::forwarderChessWrapper(receivedData, CMD_GETCHESSENGINES, server_pub);

                BYTE returnedCommand;
                vector<BYTE> expectedReturn;
                expectedReturn.push_back(CMD_GETCHESSENGINES);

                bool foundExpectedCmd = false;

                while(!foundExpectedCmd){
                    foundExpectedCmd = IRCServerNodeHelper::receiverChessWrapper(returnedProtocol, returnedCommand, answer, expectedReturn);
                    ros::spinOnce();
                }


                cout << "Data to return: ";
                for(auto &b : answer){
                    cout << (char)(int(b));
                }
                cout << endl;
                cout << "Command to send back: " << (int)foundExpectedCmd << endl;
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

                IRCServerNodeHelper::forwarderChessWrapper(receivedData, CMD_STARTCHESSENGINE, server_pub);

                BYTE returnedCommand;
                vector<BYTE> expectedReturn;
                expectedReturn.push_back(CMD_STARTCHESSENGINE);
                expectedReturn.push_back(ERROR_CMD_CHESSENGINEDOESNTEXIST);

                bool foundExpectedCmd = false;

                while(!foundExpectedCmd){
                    foundExpectedCmd = IRCServerNodeHelper::receiverChessWrapper(returnedProtocol, returnedCommand, answer, expectedReturn);
                    ros::spinOnce();
                }

                cout << "Command to send back: " << (int)foundExpectedCmd << endl;
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

                IRCServerNodeHelper::forwarderChessWrapper(receivedData, CMD_STOPCHESSENGINE, server_pub);

                BYTE returnedCommand;
                vector<BYTE> expectedReturn;
                expectedReturn.push_back(CMD_STOPCHESSENGINE);
                expectedReturn.push_back(ERROR_CMD_NOCHESSENGINERUNNING);

                bool foundExpectedCmd = false;

                while(!foundExpectedCmd){
                    foundExpectedCmd = IRCServerNodeHelper::receiverChessWrapper(returnedProtocol, returnedCommand, answer, expectedReturn);
                    ros::spinOnce();
                }

                cout << "Command to send back: " << (int)foundExpectedCmd << endl;
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

                IRCServerNodeHelper::forwarderChessWrapper(receivedData, CMD_PLAYERMOVE, server_pub);

                BYTE returnedCommand;
                vector<BYTE> expectedReturn;
                expectedReturn.push_back(CMD_PLAYERMOVE);
                expectedReturn.push_back(ERROR_CMD_NOCHESSENGINERUNNING);

                expectedReturn.push_back(ERROR_CMD_PAWNCOLLIDEDSTRAIGHT);
                expectedReturn.push_back(ERROR_CMD_PAWNCOLLIDEDDIAGONALOREMPTYCELL);
                expectedReturn.push_back(ERROR_CMD_STARTINGCELLEMPTY);
                expectedReturn.push_back(ERROR_CMD_NOTTHATCOLORSTURN);
                expectedReturn.push_back(ERROR_CMD_MOVEINVALIDORBLOCKEDBYSAMECOLOR);
                expectedReturn.push_back(ERROR_CMD_CANNOTCASTLEKINGSIDE);
                expectedReturn.push_back(ERROR_CMD_CANNOTCASTLEQUEENSIDE);
                expectedReturn.push_back(ERROR_CMD_OWNKINGINCHECK);
                expectedReturn.push_back(ERROR_CMD_OTHERKINGINCHECKMATE);
                expectedReturn.push_back(ERROR_CMD_PAWNNOTALLOWEDNOTPROMOTIONMOVE);
                expectedReturn.push_back(ERROR_CMD_PIECETOPROMOTEISNOTPAWN);
                expectedReturn.push_back(ERROR_CMD_PAWNNOTMOVINGTOENDOFBOARD);
                expectedReturn.push_back(ERROR_CMD_INVALIDPIECENAMETOPROMOTEINTO);
                expectedReturn.push_back(ERROR_CMD_INVALIDMOVEFORMAT);

                bool foundExpectedCmd = false;

                while(!foundExpectedCmd){
                    foundExpectedCmd = IRCServerNodeHelper::receiverChessWrapper(returnedProtocol, returnedCommand, answer, expectedReturn);
                    ros::spinOnce();
                }

                cout << "Command to send back: " << (int)foundExpectedCmd << endl;
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

                IRCServerNodeHelper::forwarderChessWrapper(receivedData, CMD_CHESSENGINEMOVE, server_pub);

                BYTE returnedCommand;
                vector<BYTE> expectedReturn;
                expectedReturn.push_back(CMD_CHESSENGINEMOVE);
                expectedReturn.push_back(ERROR_CMD_NOCHESSENGINERUNNING);

                expectedReturn.push_back(ERROR_CMD_PAWNCOLLIDEDSTRAIGHT);
                expectedReturn.push_back(ERROR_CMD_PAWNCOLLIDEDDIAGONALOREMPTYCELL);
                expectedReturn.push_back(ERROR_CMD_STARTINGCELLEMPTY);
                expectedReturn.push_back(ERROR_CMD_NOTTHATCOLORSTURN);
                expectedReturn.push_back(ERROR_CMD_MOVEINVALIDORBLOCKEDBYSAMECOLOR);
                expectedReturn.push_back(ERROR_CMD_CANNOTCASTLEKINGSIDE);
                expectedReturn.push_back(ERROR_CMD_CANNOTCASTLEQUEENSIDE);
                expectedReturn.push_back(ERROR_CMD_OWNKINGINCHECK);
                expectedReturn.push_back(ERROR_CMD_OTHERKINGINCHECKMATE);
                expectedReturn.push_back(ERROR_CMD_PAWNNOTALLOWEDNOTPROMOTIONMOVE);
                expectedReturn.push_back(ERROR_CMD_PIECETOPROMOTEISNOTPAWN);
                expectedReturn.push_back(ERROR_CMD_PAWNNOTMOVINGTOENDOFBOARD);
                expectedReturn.push_back(ERROR_CMD_INVALIDPIECENAMETOPROMOTEINTO);
                expectedReturn.push_back(ERROR_CMD_CHESSENGINECREATEDNOMOVE);
                expectedReturn.push_back(ERROR_CMD_INVALIDMOVEFORMAT);

                bool foundExpectedCmd = false;

                while(!foundExpectedCmd){
                    foundExpectedCmd = IRCServerNodeHelper::receiverChessWrapper(returnedProtocol, returnedCommand, answer, expectedReturn);
                    ros::spinOnce();
                }

                cout << "Data to return: ";
                for(auto &b : answer){
                    cout << (char)(int(b));
                }
                cout << endl;
                cout << "Command to send back: " << (int)foundExpectedCmd << endl;
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

    ros::Publisher server_pub = nh.advertise<ibo1_irc_api::Protocol>("ircServer_messages", 10);
    ros::Subscriber chessEngineWrapper_sub = nh.subscribe("/chessWrapper_messages", 10, &chessEngineMessageReceived);
    

    std_msgs::String messageToSendToChessEngine;
    messageToSendToChessEngine.data = "engineStart";
    server_pub.publish(messageToSendToChessEngine);
    ros::spinOnce();

    ros::Rate rate(10);

    cout << "I get to the waiting stage" << endl;

    while(server_pub.getNumSubscribers() == 0){
        rate.sleep();
    }


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
        communicationLogic(bufferSizeData, server, users, server_pub);

        cout << "\n--------------------------------------------------------" << endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}