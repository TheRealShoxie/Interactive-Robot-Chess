/*
 * UCIHandler - Class which defines a UCIHandler
 * <p>
 * This file is the implementation of the class definition in UCIHandler.h
 * Please refer to UCIHandler.h for more information.
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see UCIHandler.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

#include "ibo1_irc_api/Utility/UCIHandler.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // Constructor for initializing a uci handler and starting a process
        // with the supplied processFilePath
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

    // Checking if the engine is ready
    bool UCIHandler::isEngineReady(){
        string returnedLine = "";

        //Sending the command is ready
        subProcessHandler.write("isready");

        subProcessHandler.getLine(returnedLine);

        //Checking if returned line is readyok
        return (returnedLine.compare("readyok") == 0);

    }


    // Starting UCI interfacing with the chess engine
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
    }


    // Method to make the chess engine do a move, maniupulates the supplied chessEngineMove
    void UCIHandler::makeMove(string const &fenPosition, string const &searchSettings, string &chessEngineMove){
        string returnedLine;
        string startOfData = "bestmove ";
        bool isFinished = false;


        // Setting currentPosition
        subProcessHandler.write("position fen " + fenPosition);

        // Searching for answer from the chess engine
        subProcessHandler.write("go " +searchSettings);

        int readLineCounter = 0;

        // Keep reading till we find the best move, then extract the move and return that
        while(!isFinished){
            subProcessHandler.getLine(returnedLine);
            if(returnedLine.rfind("bestmove ", 0) == 0){
                returnedLine.erase(returnedLine.begin(), returnedLine.begin() + startOfData.size());
                chessEngineMove = DataManipulation::subString(returnedLine, " ");
                isFinished = true;
            }
            // If we still didn't find best move
            else{
                // Did we already reach the limit of read lines then break out and say not found
                if(readLineCounter > 100){
                    isFinished = true;
                    chessEngineMove = "moveNotFound!";
                }
                readLineCounter ++;
            }
        }
    }

    // Method to set a engine option
    void UCIHandler::setEngineOption(string const &optionName, string const &value){
        subProcessHandler.write("set " +optionName +" value " +value);
    }

    // Method to start a new chess game
    bool UCIHandler::startNewGame(){
        string returnedLine;
        
        subProcessHandler.write("ucinewgame");
        subProcessHandler.write("isready");
        subProcessHandler.getLine(returnedLine);

        if(returnedLine.compare("readyok") == 0) return true;
        else return false;

    }
    
    // Deconstructor for UCIHandler
    UCIHandler::~UCIHandler(){
        subProcessHandler.write("quit");
    }