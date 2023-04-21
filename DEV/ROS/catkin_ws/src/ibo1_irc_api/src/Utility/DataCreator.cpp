#include "ibo1_irc_api/Utility/DataCreator.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //
    EngineOption DataCreator::createEngineOption(string &returnedLine){

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



    void DataCreator::convertBytesToString(const vector<BYTE>& protocolData, string& stringObject){
        stringObject = string(protocolData.begin(), protocolData.end());
    }