#include <ros/ros.h>

#include <ibo1_IRC_API/Utility/FileHandler.h>

#include <std_msgs/String.h>
#include <ibo1_IRC_API/Protocol.h>


static string chessEnginesFilePathName = "/home/omar/Uni/Major_Project/Interactive-Robot-Chess/DEV/ROS/catkin_ws/src/ibo1_IRC_API/data/Chess/chessEngines.txt";
ibo1_IRC_API::Protocol returnedProtocol;

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

void serverMessageReceived(const ibo1_IRC_API::Protocol& msg){
    ROS_INFO("I receive here ChessEngineWrapper!");
    returnedProtocol = msg;
}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // //////////////////// //
    // Internal Functions.  //
    // //////////////////// //


void wrapperLogic(vector<ChessEngineDefinitionStruct>& chessEngines, ros::Publisher *chessWrapper_pub){

    BYTE test;

}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "chessWrapper");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher chessWrapper_pub = nh.advertise<ibo1_IRC_API::Protocol>("chessWrapper_messages", 10);
    ros::Subscriber server_sub = nh.subscribe("/ircServer_messages", 10, &serverMessageReceived);

    vector<ChessEngineDefinitionStruct> chessEngines = FileHandler::readChessEngines(chessEnginesFilePathName);


    ros::Rate rate(10);

    // Waiting till ros system subscribers are activated
    while(chessWrapper_pub.getNumSubscribers() == 0){
        rate.sleep();
    }

    ibo1_IRC_API::Protocol test;
    test.cmd = (BYTE)0x02;
    vector<BYTE> dataTest;
    dataTest.push_back((BYTE)0x02); 
    dataTest.push_back((BYTE)0x03); 
    dataTest.push_back((BYTE)0x04); 
    dataTest.push_back((BYTE)0x05); 
    dataTest.push_back((BYTE)0x06); 
    test.data = dataTest;

    chessWrapper_pub.publish(test);

    while(ros::ok()){

        

        wrapperLogic(chessEngines, &chessWrapper_pub);



        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}