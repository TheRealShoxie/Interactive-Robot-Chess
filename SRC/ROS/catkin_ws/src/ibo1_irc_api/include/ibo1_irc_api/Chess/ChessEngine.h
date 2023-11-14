#ifndef CHESSENGINE_H
#define CHESSENGINE_H


/*
 * ChessEngine - Class which represents a chessengine
 * <p>
 * Used as a chess engine representation. This class defines what a chess engine is. It holds ChessBoard.
 * It is used for starting a specified chess engine with its filepath. Used for interacting with the chessboard and getting 
 * chess moves from a supplied chess engine. It uses UCIHandler for communicating with the chess engine and starting it.
 * 
 * ChessEngine does not deal with bad filepath input this needs to be done outside of it before calling the constructor
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see ChessBoard.h
 * @see UCIHandler.h
 * @see ChessWrapperNode.cpp
 * @see ChessEngine.cpp
 * @see InternalProtocolDefinition.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //


#include <ibo1_irc_api/Utility/UCIHandler.h>
#include <ibo1_irc_api/Chess/ChessBoard.h>
#include <ibo1_irc_api/ProtocolAPI/InternalProtocolDefinition.h>



    // ////////// //
    // Constants. //
    // ////////// //

class ChessEngine{
    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // Constructor to initialize a ChessEngine with a supplied filepath to the chessengine to be started
        ChessEngine(string const &processFilePathName);

        // //////// //
        // Methods. //
        // //////// //

        // Method used for making a player move
        void playerMove(BYTE &returnedProtocolByte, string &move);

        // Method used for making the chess engine do a move
        void chessEngineMove(BYTE &returnedProtocolByte, string &chessEngineMove);

        // Method used for setting the ChessEngine options
        void setChessEngineOption(string const &optionName, string const &value);

        // Method used for starting a new game
        void startNewGame();

        // Method used for getting a FEN representation of the internal chess board
        string getChessBoardFENString();

        // Method used for getting a 2d string representation of the internal chess board
        string getChessBoardString();
        
        // Method for comparing two chess engines
        // Overrides default = operator
        ChessEngine& operator= (ChessEngine&&){ return *this; }

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // Method used for setting the search options when making a chess engine move
        void setSearchOptions(string const &setSearchOptions);

        // Method getting the current search options used
        void getSearchOptions(string &getSearchOptions);

        // Method used to get the possible chess engine options from the chess engine
        void getChessEngineOptions(vector<EngineOption> &engineOptions);

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //

        // Method used to check if the chess engine was started
        bool getChessEngineStarted();

        // Method used for checking if last move was castle move
        bool wasLastMoveCastleMove(){
            return chessBoard.wasLastMoveCastleMove();
        }

    private:

        // ////////////// //
        // Class methods. //
        // ////////////// //

        // Method converts moveCodes from the ChessBoard Class into internal protocol cmds
        void getInternalProtocolCode(int moveCode, BYTE const &defaultValue, BYTE &returnByte);
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //
        UCIHandler uciHandler;

        ChessBoard chessBoard;

        string searchOptions = "";
        vector<EngineOption> engineOptions;
        bool chessEngineStarted = false;

};
#endif //CHESSENGINE_H