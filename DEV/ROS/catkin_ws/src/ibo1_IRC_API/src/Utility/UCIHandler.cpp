#include "ibo1_IRC_API/UCIHandler.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    UCIHandler::UCIHandler(){}
    
    UCIHandler::UCIHandler(string const &processFilePathName)
        :subProcessHandler(processFilePathName){
    }

    // ////////////// //
    // Class methods. //
    // ////////////// //

    EngineOption UCIHandler::createEngineOption(string &returnedLine){

        string startOfData = "option name ";
        string typeSplit = " type ";
        string defaultSplit = " default";
        string minSplit = " min ";
        string maxSplit = " max ";
        int end = 0;
            
        EngineOption engineOption;

        // Deleting start of the option String
        returnedLine.erase(returnedLine.begin(), returnedLine.begin() + startOfData.size());

        engineOption.name = dm.subString(returnedLine, typeSplit);
        
        end = returnedLine.find(defaultSplit);
        if(end == -1){
            engineOption.typeOfValue = returnedLine;
            return engineOption;
        }

        engineOption.typeOfValue = dm.subString(returnedLine, defaultSplit);

        // Checking if there is content after the
        if(returnedLine.size() > 0){
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

                    engineOption.minValue = dm.subString(returnedLine, maxSplit);

                    engineOption.maxValue = returnedLine;
                }

            }
                        
        }

        return engineOption;
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    bool UCIHandler::isEngineReady(){
        string returnedLine = "";
        subProcessHandler.getLine(returnedLine);

        return returnedLine.compare("readyok");

    }

    bool UCIHandler::startUCI(vector<EngineOption> &engineOptions){
        EngineOption engineOption;
        string returnedLine;
        bool isFinished = false;
        int end;
        subProcessHandler.getLine(returnedLine);

        subProcessHandler.write("uci");

        //int maxReads = 10000;
        int i = 0;


        while(!isFinished){
            subProcessHandler.getLine(returnedLine);
            if(returnedLine.compare("uciok") == 0){
                isFinished = true;
                break;
            }

            if(returnedLine.rfind("option", 0) == 0){
                engineOption = createEngineOption(returnedLine);

                if(engineOption.name.size() > 0){
                    engineOptions.push_back(engineOption);
                }
            }
            i ++;
        }
        return true;

        //Case for when it needs to false? Maybe if uciok doesnt show up after a certain amount of time?
        //return false;
    }

    string UCIHandler::makeMove(string const &fenPosition, string const &searchSettings){
        string returnedLine;
        string startOfData = "bestmove ";
        string move = "";
        bool isFinished = false;
        
        subProcessHandler.write("position " + fenPosition);
        subProcessHandler.write("go " +searchSettings);

        while(!isFinished){
            subProcessHandler.getLine(returnedLine);
            if(returnedLine.rfind("bestmove ", 0) == 0){

                returnedLine.erase(returnedLine.begin(), returnedLine.begin() + startOfData.size());
                move = dm.subString(returnedLine, " ");
                isFinished = true;
            }
        }

        return move;
    }

    void UCIHandler::setEngineOptions(string const &optionName, string const &value){
        subProcessHandler.write("set " +optionName +" value " +value);
    }

    bool UCIHandler::startNewGame(){
        string returnedLine;
        
        subProcessHandler.write("ucinewgame");
        subProcessHandler.write("isready");
        subProcessHandler.getLine(returnedLine);

        if(returnedLine.compare("readyok") == 0) return true;
        else return false;

    }
    
    void UCIHandler::closeProcess(){
        subProcessHandler.write("quit");
        subProcessHandler.closeSubProcess();
    }