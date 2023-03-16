#include <ros/ros.h>
#include <std_msgs/String.h>

#include "ibo1_IRC_API/IRCServer.h"





void communicationLogic(int bufferSizeData, IRCServer *server){
    std::vector<BYTE> returnData;
    std::vector<BYTE> answer;

    
    if(bufferSizeData > 0) returnData = server->getData(bufferSizeData);



    if(!returnData.empty()){
        std::cout << "CLIENT COMMAND is:" << server->getClientCommand() << std::endl;
        std::cout << "Following Data has been receieved: ";
        for(BYTE data : returnData){
            std::cout << data << "-";
        }

        switch (server->getClientCommand())
        {
            case CMD_CONNECT:
                break;

            case CMD_STRING: {
                std::cout << "Sending string back!" <<std::endl;
                std::string returnString = "Acknowledge!";
                std::copy(returnString.begin(), returnString.end(), std::back_inserter(answer));

                break;  
            }

            case CMD_LOGIN: {

                
                break;
            }
                
            default:
                return;
                break;
        }

    }
    

    server->commandAnswer(answer);

}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/


int main (int argc, char **argv){
    ros::init(argc, argv, "publish_Server");
    ros::NodeHandle nh;

    ros::Publisher server_pub = nh.advertise<std_msgs::String>("/server_messages", 10);

    IRCServer server = IRCServer(54001);
    server.initiateServerSocket();
    std::cout << server.getClientSocket() << std::endl;

    int bufferSizeData = 0;

    ros::Rate rate(1);
    while(ros::ok()){


        bufferSizeData = server.commandExtraction();

        std::cout << "Size of to come Data: " << bufferSizeData << std::endl;
        communicationLogic(bufferSizeData, &server);

        rate.sleep();
    }

    delete(&server);

    return 0;
}