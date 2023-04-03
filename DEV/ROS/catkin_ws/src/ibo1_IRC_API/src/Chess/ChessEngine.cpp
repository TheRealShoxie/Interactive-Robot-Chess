#include "ibo1_IRC_API/Chess/ChessEngine.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

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

    void ChessEngine::getProtocolCode(int moveCode, BYTE  const &defaultValue, BYTE &returnByte){
        switch (moveCode)
        {
        case -1:
            returnByte = ERROR_CMD_PAWNCOLLIDEDSTRAIGHT; 
            break;

        case -2:
            returnByte = ERROR_CMD_PAWNCOLLIDEDDIAGONALOREMPTYCELL; 
            break;

        case -3:
            returnByte = ERROR_CMD_STARTINGCELLEMPTY; 
            break;

        case -4:
            returnByte = ERROR_CMD_NOTTHATCOLORSTURN; 
            break;

        case -5:
            returnByte = ERROR_CMD_MOVEINVALIDORBLOCKEDBYSAMECOLOR; 
            break;

        case -6:
            returnByte = ERROR_CMD_CANNOTCASTLEKINGSIDE; 
            break;

        case -7:
            returnByte = ERROR_CMD_CANNOTCASTLEQUEENSIDE; 
            break;

        case -8:
            returnByte = ERROR_CMD_INVALIDMOVEFORMAT; 
            break;
        
        default:
            returnByte = defaultValue;
            break;
        }
    }


    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    void ChessEngine::setSearchOptions(string const &setSearchOptions){
        searchOptions = setSearchOptions;
    }

    void ChessEngine::getSearchOptions(string &getSearchOptions){
        getSearchOptions = searchOptions;
    }

    void ChessEngine::getChessEngineOptions(vector<EngineOption> &chessEngineOptions){
        chessEngineOptions = engineOptions;
    }

    void ChessEngine::setChessEngineOptions(string const &optionName, string const &value){
        uciHandler.setEngineOptions(optionName, value);
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    bool ChessEngine::getChessEngineStarted(){
        return chessEngineStarted;
    }

    // //////// //
    // Methods. //
    // //////// //
    void ChessEngine::playerMove(BYTE &returnedProtocolByte, string &move){

        string chessEngineMove = "";

        int returnedMoveCode = chessBoard->move(move);

        BYTE returnedCommand;
        
        getProtocolCode(returnedMoveCode, CMD_PLAYERMOVE, returnedProtocolByte);
    }

    void ChessEngine::chessEngineMove(BYTE &returnedProtocolByte, string &chessEngineMove){
        
        cout << "ChessEngineMove Try to get fenString! "<< endl;

        // Getting the current Fen representation of the board
        string currentFENPosition = chessBoard->toFENString();

        cout << "ChessEngineMove Try make the chessEngine move! "<< endl;
        cout << "UCIHandler memory address: " << &uciHandler << endl;

        // Making the ChessEngine make the move.
        uciHandler.makeMove(currentFENPosition, searchOptions, chessEngineMove);

        cout << "ChessEngine made a move and the move is: " << chessEngineMove << endl;

        // Setting the move of the chessEngine to the internal board.
        int returnedMoveCode = chessBoard->move(chessEngineMove);

        cout << "Tried the move and we got following code: " << returnedMoveCode << endl;

        
        getProtocolCode(returnedMoveCode, CMD_CHESSENGINEMOVE, returnedProtocolByte);
    }

    void ChessEngine::startNewGame(){
        cout << "UCIHandler memory address: " << &uciHandler << endl;
        if(!(uciHandler.startNewGame())) throw runtime_error("Couldn't start a fresh game!");
        chessBoard = new ChessBoard();
    }

    void ChessEngine::closeEngine(){
        uciHandler.closeProcess();
    }

    string ChessEngine::getChessBoardFENString(){

        return chessBoard->toFENString();
    }

    string ChessEngine::getChessBoardString(){
        return chessBoard->toString();
    }