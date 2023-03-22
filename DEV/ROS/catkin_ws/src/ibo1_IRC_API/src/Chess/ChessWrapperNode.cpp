#include <ros/ros.h>

#include <std_msgs/String.h>
#include <ibo1_IRC_API/Chess.h>


static string chessEngineFilePathName = "src/ibo1_IRC_API/data/Chess/stockfish/src/stockfish";

int main (int argc, char **argv){
    ros::init(argc, argv, "chessWrapper");
    ros::NodeHandle nh;

    ros::Publisher chessWrapper_pub = nh.advertise<std_msgs::String>("/chessWrapper_messages", 1);


    //Following code to return 0 is just a test for classes and will all be written inside the CHess.cpp
    UCIHandler uciHandler(chessEngineFilePathName);

    vector<EngineOption> engineOptions;

    uciHandler.startUCI(engineOptions);

    // for(auto &engineOption : engineOptions){
    //     cout << "----------------------------------------------" << endl;
    //     cout << "Option name: " << engineOption.name << endl;
    //     cout << "Type of value: " << engineOption.typeOfValue << endl;
    //     cout << "Default value: " << engineOption.defaultValue << endl;
    //     cout << "Min value: " << engineOption.minValue << endl;
    //     cout << "Max value: " << engineOption.maxValue << endl;
    //     cout << "Rest Values: " << engineOption.restValues << endl;
    //     cout << "----------------------------------------------" << endl;
    //     cout << "\n";
    // }
    //Chess chess(chessEngineFilePathName);

    string moveToMake = uciHandler.makeMove("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1", "depth 20");
    cout << "Move the Engine said:-" << moveToMake << "-" << endl;

    uciHandler.closeProcess();
    

    return 0;

    ros::Rate rate(1);
    while(ros::ok()){



        rate.sleep();
    }


    return 0;
}