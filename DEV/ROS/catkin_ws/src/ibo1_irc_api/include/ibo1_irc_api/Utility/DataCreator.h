#ifndef DATACREATOR_H
#define DATACREATOR_H

/*
 * DataCreator
 * <p>
 * This file defines the DataCreator Class and the EngineOption struct.
 * The EngineOption struct is in multiple stages of the system to set and get engine options from the chess engine.
 * 
 * The DataCreator helps in the creation of creating a chess engine from a output from the chess engine. which also uses the
 * DataManipulation class methods
 * Converting a vector<BYTE> to a string
 * string to bool
 * 
 * All DataCreator methods are static for easy access.
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see UCIHandler.h
 * @see ChessEngine.h
 * @see DataManipulation.h
 * @see IRCServerNode.cpp
 * @see SystemStateMachineNode.cpp
 * @see RobotArmStateMachineNode.cpp
 * @see ChessWrapperNode.cpp
 * @see FileHandler.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //
   
#include "ibo1_irc_api/Utility/DataManipulation.h"
#include "ibo1_irc_api/User/User.h"
#include <string>

using namespace std;

    // ////////// //
    // Structs.   //
    // ////////// //
   
   // Struct definition for a EngineOption
   struct EngineOption{
        string name = "";
        string typeOfValue;
        string defaultValue;
        string minValue;
        string maxValue;
        string restValues;
    };

    // Struct definition for chess engine
    struct ChessEngineDefinitionStruct{
        string name = "";
        string filePathName = "";

        inline bool operator==(const ChessEngineDefinitionStruct& e) const{
            return (name.compare(e.name) == 0 && filePathName.compare(e.filePathName) == 0);
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

        // Converts string to bool. Throws an invalid argument error
        static bool stringToBool(const string& stringToConvert){
            
            // Checking if the string is a 1
            if(stringToConvert.compare("1") == 0) return true;
            // Checking if the string is a 0
            else if(stringToConvert.compare("0") == 0) return false;

            // Checking if the string is a true
            else if(stringToConvert.compare("true") == 0) return true;

            // Checking if the string is a false
            else if(stringToConvert.compare("false") == 0) return false;

            // Otherwise does not map to one of the possibilities thus we thro an invalid argument error
            else{
                throw std::invalid_argument(stringToConvert + " is not convertable to bool");
            }
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