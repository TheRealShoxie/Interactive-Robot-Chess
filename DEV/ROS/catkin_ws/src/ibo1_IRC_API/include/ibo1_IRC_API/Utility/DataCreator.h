#ifndef DATACREATOR_H
#define DATACREATOR_H
   
#include "ibo1_IRC_API/Utility/DataManipulation.h"
#include "ibo1_IRC_API/User/User.h"
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

        inline bool operator==(const EngineOption& e) const{
            return (name.compare(e.name) == 0 
                    && typeOfValue.compare(e.typeOfValue) == 0 
                    && defaultValue.compare(e.defaultValue) == 0 
                    && minValue.compare(e.minValue) == 0 
                    && maxValue.compare(e.maxValue) == 0 
                    && restValues.compare(e.restValues) == 0);
        }
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