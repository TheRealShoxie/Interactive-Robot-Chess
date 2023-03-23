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

        subProcessHandler.write("isready");

        subProcessHandler.getLine(returnedLine);

        return (returnedLine.compare("readyok") == 0);

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
                engineOption = DataCreator::createEngineOption(returnedLine);

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
                chessEngineMove = DataManipulation::subString(returnedLine, " ");
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