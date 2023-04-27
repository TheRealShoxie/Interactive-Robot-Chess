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
        static EngineOption createEngineOption(string &returnedLine){
            string startOfData = "option name ";
            string typeSplit = " type ";
            string defaultSplit = " default";
            string minSplit = " min ";
            string maxSplit = " max ";
            int end = 0;
                
            EngineOption engineOption;

            // Deleting start of the option String
            returnedLine.erase(returnedLine.begin(), returnedLine.begin() + startOfData.size());

            // Getting the name of the option
            engineOption.name = DataManipulation::subString(returnedLine, typeSplit);

            
            end = returnedLine.find(defaultSplit);

            //Check if default exists otherwise assign typeOfValue and return
            if(end == -1){
                engineOption.typeOfValue = returnedLine;
                return engineOption;
            }

            //Extracting the typeOfValue
            engineOption.typeOfValue = DataManipulation::subString(returnedLine, defaultSplit);

            // Checking if there is content after the default
            if(returnedLine.size() > 0){

                // Deleting the white space that is left over and find the next white space splitter
                returnedLine.erase(returnedLine.begin(), returnedLine.begin() + 1);
                end = returnedLine.find(" ");

                // Checking if we have more content after the default value, if not then default options is the rest
                if(end == -1){
                    engineOption.defaultValue = returnedLine;
                } else{
                    engineOption.defaultValue = returnedLine.substr(0, end);

                    end = returnedLine.find(minSplit);
                    // Checking if the rest of the split is by min thus also by max
                    if(end == -1){
                        engineOption.restValues = returnedLine;
                    } else{
                        returnedLine.erase(returnedLine.begin(), returnedLine.begin() + end + minSplit.size());

                        engineOption.minValue = DataManipulation::subString(returnedLine, maxSplit);

                        engineOption.maxValue = returnedLine;
                    }

                }
                            
            }

            return engineOption;
        }


        // Converts my vector BYTE to a string Object. manipulates the passed objects
        static void convertBytesToString(const vector<BYTE>& data, string& stringObject){
            stringObject = string(data.begin(), data.end());
        }

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