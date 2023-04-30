/*
 * ChessEngine - Class which represents a chess engine
 * <p>
 * This file is the implementation of the class definition in ChessEngine.h
 * Please refer to ChessEngine.h for more information.
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see ChessEngine.h
*/

// Include for the header file it overrides the functions of
#include "ibo1_irc_api/Chess/ChessEngine.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // Default constructor from chess engine
    ChessEngine::ChessEngine(string const &processFilePathName)
        :uciHandler(processFilePathName), searchOptions("depth 10"){

        // Initializing engine
        chessEngineStarted = true;
        uciHandler.startUCI(engineOptions);
        if(!(uciHandler.isEngineReady())) throw runtime_error("Engine was not ready!");
        startNewGame();
    }

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // Converts the moveCodes into its corresponding internal protocol code
    void ChessEngine::getInternalProtocolCode(int moveCode, BYTE  const &defaultValue, BYTE &returnByte){
        switch (moveCode)
        {
        case -1:
            returnByte = ERROR_INTERNAL_CMD_PAWNCOLLIDEDSTRAIGHT; 
            break;

        case -2:
            returnByte = ERROR_INTERNAL_CMD_PAWNCOLLIDEDDIAGONALOREMPTYCELL; 
            break;

        case -3:
            returnByte = ERROR_INTERNAL_CMD_STARTINGCELLEMPTY; 
            break;

        case -4:
            returnByte = ERROR_INTERNAL_CMD_NOTTHATCOLORSTURN; 
            break;

        case -5:
            returnByte = ERROR_INTERNAL_CMD_MOVEINVALIDORBLOCKEDBYSAMECOLOR; 
            break;

        case -6:
            returnByte = ERROR_INTERNAL_CMD_CANNOTCASTLEKINGSIDE; 
            break;

        case -7:
            returnByte = ERROR_INTERNAL_CMD_CANNOTCASTLEQUEENSIDE; 
            break;

        case -8:
            returnByte = ERROR_INTERNAL_CMD_OWNKINGINCHECK;
            break;

        case -9:
            returnByte = ERROR_INTERNAL_CMD_OTHERKINGINCHECKMATE;
            break;

        case -10:
            returnByte = ERROR_INTERNAL_CMD_PAWNNOTALLOWEDNOTPROMOTIONMOVE;
            break;

        case -11:
            returnByte = ERROR_INTERNAL_CMD_PIECETOPROMOTEISNOTPAWN; 
            break;

        case -12:
            returnByte = ERROR_INTERNAL_CMD_PAWNNOTMOVINGTOENDOFBOARD; 
            break;

        case -13:
            returnByte = ERROR_INTERNAL_CMD_INVALIDPIECENAMETOPROMOTEINTO; 
            break;

        case -14:
            returnByte = ERROR_INTERNAL_CMD_INVALIDMOVEFORMAT; 
            break;
        
        default:
            returnByte = defaultValue;
            break;
        }
    }


    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // Used for setting search options for the chess engine
    void ChessEngine::setSearchOptions(string const &setSearchOptions){
        searchOptions = setSearchOptions;
    }



    // Used for getting possible search options for the chess engine
    void ChessEngine::getSearchOptions(string &getSearchOptions){
        getSearchOptions = searchOptions;
    }



    // Used for getting the possible chessEngine overall options
    void ChessEngine::getChessEngineOptions(vector<EngineOption> &chessEngineOptions){
        chessEngineOptions = engineOptions;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // Used for checking if the chessEngine is started
    bool ChessEngine::getChessEngineStarted(){
        return chessEngineStarted;
    }

    // //////// //
    // Methods. //
    // //////// //

    // Used for making a player move
    void ChessEngine::playerMove(BYTE &returnedProtocolByte, string &move){

        // Make a player move and convert the moveCode into a internal protocol code
        int returnedMoveCode = chessBoard.move(move);

        BYTE returnedCommand;
        
        getInternalProtocolCode(returnedMoveCode, CMD_INTERNAL_PLAYERMOVE, returnedProtocolByte);
    }



    // Make a chess engine move and convert the moveCode into a internal protocol code
    void ChessEngine::chessEngineMove(BYTE &returnedProtocolByte, string &chessEngineMove){

        // Getting the current Fen representation of the board
        string currentFENPosition = chessBoard.toFENString();

        // Making the ChessEngine make the move.
        uciHandler.makeMove(currentFENPosition, searchOptions, chessEngineMove);

        // Checking if chessEngine did not find a move otherwise return chess engine created no move
        if(chessEngineMove.compare("moveNotFound!") == 0){
            returnedProtocolByte = ERROR_INTERNAL_CMD_CHESSENGINECREATEDNOMOVE;
        }else{
            // Setting the move of the chessEngine to the internal board.
            int returnedMoveCode = chessBoard.move(chessEngineMove);

            
            getInternalProtocolCode(returnedMoveCode, CMD_INTERNAL_CHESSENGINEMOVE, returnedProtocolByte);
        }
    }



    //Setting a chess engine option
    void ChessEngine::setChessEngineOption(string const &optionName, string const &value){
        uciHandler.setEngineOption(optionName, value);
    }



    // Starting a new game
    void ChessEngine::startNewGame(){
        if(!(uciHandler.startNewGame())) throw runtime_error("Couldn't start a fresh game!");
        chessBoard = ChessBoard();
    }



    // Getting the FEN representation of the chess board
    string ChessEngine::getChessBoardFENString(){
        return chessBoard.toFENString();
    }



    // Getting a 2d string representation of the chess board
    string ChessEngine::getChessBoardString(){
        return chessBoard.toString();
    }