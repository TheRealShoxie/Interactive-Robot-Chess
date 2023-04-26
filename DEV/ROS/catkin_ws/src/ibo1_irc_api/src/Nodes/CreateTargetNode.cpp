#include <ros/ros.h>

#include <ibo1_irc_api/Protocol.h>
#include <ibo1_irc_api/ChessCells.h>


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //
// Image sizes
static const int x_max = 1280;
static const int y_max = 720;

// For ros subscribers & publisher
ibo1_irc_api::Protocol returnedProtocol;
ibo1_irc_api::ChessCells returnedChessCells;

ros::Publisher* commandExecuter_pub_ptr;


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //
void createTargetMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    cout << "----------------------------------------------------------" << endl;
    cout << "I received following on createTarget: " << endl;
    cout << "I received from: " << (int)returnedProtocol.sender << endl;
    cout << "CmdByte: " << (int)returnedProtocol.cmd << endl;
    cout << "----------------------------------------------------------" << endl;
}

void chessCellDetectionMessageReceived(const ibo1_irc_api::ChessCells& msg){
    returnedChessCells = msg;
    cout << "----------------------------------------------------------" << endl;
    cout << "I received following on createTarget: " << endl;
    cout << "----------------------------------------------------------" << endl;
}



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Methods.   //
    // ////////// //



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "createTarget");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber server_sub = nh.subscribe("/ircCreateTarget", 1, &createTargetMessageReceived);
    ros::Subscriber server_sub = nh.subscribe("/chessCellDetection", 1, &chessCellDetectionMessageReceived);

    ros::Publisher commandExecuter_pub = nh.advertise<ibo1_irc_api::Protocol>("ircCommandExecuter", 10);

    commandExecuter_pub_ptr = &commandExecuter_pub;

    ros::Rate rate(10);

    while(ros::ok()){


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}