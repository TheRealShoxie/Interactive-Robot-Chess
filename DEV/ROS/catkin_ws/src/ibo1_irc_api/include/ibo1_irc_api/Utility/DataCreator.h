#ifndef DATACREATOR_H
#define DATACREATOR_H
   
#include "ibo1_irc_api/Utility/DataManipulation.h"
#include "ibo1_irc_api/User/User.h"
#include <string>

using namespace std;

    // ////////// //
    // Structs.   //
    // ////////// //
   
   struct EngineOption{
        string name = "";
        string typeOfValue;
        string defaultValue;
        string minValue;
        string maxValue;
        string restValues;
    };
   
    // ////////// //
    // Constants. //
    // ////////// //

class DataCreator{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //
        // method to create a engineOption out of the uci option string
        static EngineOption createEngineOption(string &returnedLine);
        static void convertBytesToString(const vector<BYTE>& receivedData, string& chessEngineName);

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //

    private:

        // ////////////// //
        // Class methods. //
        // ////////////// //
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //

    
};
#endif //DATACREATOR_H