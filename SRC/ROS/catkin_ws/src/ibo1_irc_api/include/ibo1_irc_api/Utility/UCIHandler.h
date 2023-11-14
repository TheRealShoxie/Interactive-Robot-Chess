#ifndef UCIHANDLER_H
#define UCIHANDLER_H

/*
 * UCIHandler
 * <p>
 * The UCIHandler class is used to interact with a chess engine that uses the UCI protocol
 * The UCI protocol is a Universal Chess Interface for chess engines.
 * The UCI was developed by Rudolf Huber and Stefan Meyer-Kahlen and released in November 2000.
 * A link to the protocol definition: http://page.mi.fu-berlin.de/block/uci.htm
 * Accessed: 30th of April 2023
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see ChessEngine.h
 * @see SubProcessHandler.
 * @see DataCreator.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

#include <ibo1_irc_api/Utility/SubProcessHandler.h>
#include <ibo1_irc_api/Utility/DataCreator.h>
   
    // ////////// //
    // Structs.   //
    // ////////// //

    // ////////// //
    // Constants. //
    // ////////// //

class UCIHandler{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // Constructor for initializing a uci handler and starting a process
        // with the supplied processFilePath
        UCIHandler(string const &processFilePathName);

        // //////// //
        // Methods. //
        // //////// //

        // Checking if the engine is ready
        bool isEngineReady();

        // Starting UCI interfacing with the chess engine
        bool startUCI(vector<EngineOption> &engineOptions);

        // Method to make the chess engine do a move, maniupulates the supplied chessEngineMove
        void makeMove(string const &fenPosition, string const &searchSettings, string &chessEngineMove);
        
        // Method to set a engine option
        void setEngineOption(string const &optionName, string const &value);
        
        // Method to start a new chess game
        bool startNewGame();

        // Deconstructor for UCIHandler
        ~UCIHandler();

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
        SubProcessHandler subProcessHandler;

    
};
#endif //UCIHANDLER_H