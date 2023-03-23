#include "ibo1_IRC_API/Utility/UCIHandler.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    UCIHandler::UCIHandler(){}
    
    UCIHandler::UCIHandler(string const &processFilePathName)
        :subProcessHandler(processFilePathName){

        // Absorbing the first line that is sent by the chess engine when it starts.
        string returnedLine;
        subProcessHandler.getLine(returnedLine);
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

        // Getting the name of the option
        engineOption.name = dm.subString(returnedLine, typeSplit);

        
        end = returnedLine.find(defaultSplit);

        //Check if default exists otherwise assign typeOfValue and return
        if(end == -1){
            engineOption.typeOfValue = returnedLine;
            return engineOption;
        }

        //Extracting the typeOfValue
        engineOption.typeOfValue = dm.subString(returnedLine, defaultSplit);

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

        // Sending the uci command
        subProcessHandler.write("uci");


        //Reading options and creating engineOptions till uciok is read.
        while(!isFinished){
            // Getting the line from the stream
            subProcessHandler.getLine(returnedLine);

            // Checking for uciok
            if(returnedLine.compare("uciok") == 0){
                isFinished = true;
                break;
            }

            // Checking if there is an option, if yes create engineOption and add to the vector. 
            if(returnedLine.rfind("option", 0) == 0){
                engineOption = createEngineOption(returnedLine);

                if(engineOption.name.size() > 0){
                    engineOptions.push_back(engineOption);
                }
            }
        }
        return true;

        //Case for when it needs to false? Maybe if uciok doesnt show up after a certain amount of time?
        //return false;
    }

    void UCIHandler::makeMove(string const &fenPosition, string const &searchSettings, string &chessEngineMove){
        string returnedLine;
        string startOfData = "bestmove ";
        bool isFinished = false;

        // Setting currentPosition
        subProcessHandler.write("position " + fenPosition);

        // Searching for answer from the chess engine
        subProcessHandler.write("go " +searchSettings);

        // Keep reading till we find the best move, then extract the move and return that
        while(!isFinished){
            subProcessHandler.getLine(returnedLine);
            if(returnedLine.rfind("bestmove ", 0) == 0){

                returnedLine.erase(returnedLine.begin(), returnedLine.begin() + startOfData.size());
                chessEngineMove = dm.subString(returnedLine, " ");
                isFinished = true;
            }
        }
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