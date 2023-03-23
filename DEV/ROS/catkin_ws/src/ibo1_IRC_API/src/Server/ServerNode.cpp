#include <ros/ros.h>

#include "ibo1_IRC_API/Server/IRCServer.h"
#include "ibo1_IRC_API/DataObjects/User.h"
#include "ibo1_IRC_API/DataObjects/Users.h"
#include <std_msgs/String.h>


    // ////////// //
    // Constants. //
    // ////////// //
static string usersFilePathName = "src/ibo1_IRC_API/data/Users/users.txt";


void communicationLogic(int bufferSizeData, IRCServer *server, Users *users){
    std::vector<BYTE> receivedData;
    std::vector<BYTE> answer;

    if(bufferSizeData == -1){
        server->setClientCommand(ERROR_CONNECT);
    }
    else if(bufferSizeData > 0 && bufferSizeData < 1000){
        receivedData = server->getData(bufferSizeData);

        cout << "Client connected: " << to_string(server->getClientConnected()) << endl;
        cout << "CLIENT COMMAND is:" << server->getClientCommand() << endl;
        /*cout << "Size of return Data: " << to_string(returnData.size()) << endl; 
        cout << "Following Data has been received: " << endl;
        for(BYTE data : returnData){
            std::cout << data << "-";
        }*/

        switch (server->getClientCommand())
        {
            case CMD_CONNECT:
                break;

            case CMD_LOGIN: {
                User user = User(receivedData);
                try{
                    User foundUser = users->findUser(user);
                    cout << "User: \n" << foundUser.toString() << endl;
                    cout << foundUser.isAdmin() << endl;
                    if(foundUser.isAdmin()) answer.push_back(0x01);
                    else answer.push_back(0x00);
                }catch(runtime_error er){
                    cout << "User was not found!" << endl;
                    server->setClientCommand(ERROR_CMD_USERDOESNTEXIST);
                }
                break;
            }
                
            default:
                return;
                break;
        }

    }

    server->sendAnswer(answer);

}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/


int main (int argc, char **argv){
    ros::init(argc, argv, "ircServer");
    ros::NodeHandle nh;

    ros::Publisher server_pub = nh.advertise<std_msgs::String>("/ircServer_messages", 10);
    Users users(usersFilePathName);

    IRCServer server = IRCServer(54001);
    server.initiateServerSocket();
    cout << "Client Socket: " << server.getClientSocket() << endl;

    int bufferSizeData = 0;

    ros::Rate rate(10);
    while(ros::ok()){


        bufferSizeData = server.commandExtraction();

        cout << "Size of to come Data: " << bufferSizeData << endl;
        communicationLogic(bufferSizeData, &server, &users);

        rate.sleep();
    }

    delete(&server);

    return 0;
}