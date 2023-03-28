#include "ibo1_IRC_API/Chess/ChessEngine.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    ChessEngine::ChessEngine(string const &processFilePathName)
        :uciHandler(processFilePathName), searchOptions("depth 10"), wholeMoves(0), colorTurn('w'),
        currentFENPosition("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"),
        dataChecker(){

        // Initializing engine
        chessEngineStarted = true;
        uciHandler.startUCI(engineOptions);
        if(!(uciHandler.isEngineReady())) throw runtime_error("Engine was not ready!");
        startNewGame();
    }

    // ////////////// //
    // Class methods. //
    // ////////////// //

    void ChessEngine::updateBoardState(string const &move){

        //Increasing move turner
        wholeMoves++;

        // Checking if the move is white castling king side and it is still allowed, then castle
        if(move.compare("g1f1") == 0 && castleRights[0][0]){
            int originalWhiteKingPosition = 60;
            int originalWhiteRookPosition = 63;
            int newWhiteKingPosition = 62;
            int newWhiteRookPosition = 61;

            char pieceWhiteKing = chessBoard[originalWhiteKingPosition];
            char pieceWhiteRook = chessBoard[originalWhiteRookPosition];
            chessBoard[originalWhiteKingPosition] = '-';
            chessBoard[originalWhiteRookPosition] ='-';
            chessBoard[newWhiteKingPosition] = pieceWhiteKing;
            chessBoard[newWhiteRookPosition] = pieceWhiteRook;

            castleRights[0][0] = false;
            
        }
        // Checking if the move is white castling queen side and it is still allowed, then castle
        else if(move.compare("c1d1") == 0 && castleRights[0][1]){
            int originalWhiteKingPosition = 60;
            int originalWhiteRookPosition = 56;
            int newWhiteKingPosition = 58;
            int newWhiteRookPosition = 59;

            char pieceWhiteKing = chessBoard[originalWhiteKingPosition];
            char pieceWhiteRook = chessBoard[originalWhiteRookPosition];
            chessBoard[originalWhiteKingPosition] = '-';
            chessBoard[originalWhiteRookPosition] ='-';
            chessBoard[newWhiteKingPosition] = pieceWhiteKing;
            chessBoard[newWhiteRookPosition] = pieceWhiteRook;

            castleRights[0][1] = false;

        }
        // Checking if the move is black castling king side and it is still allowed, then castle
        else if(move.compare("g8f8") == 0 && castleRights[1][0]){
            int originalBlackKingPosition = 4;
            int originalBlackRookPosition = 7;
            int newBlackKingPosition = 6;
            int newBlackRookPosition = 5;

            char pieceWhiteKing = chessBoard[originalBlackKingPosition];
            char pieceWhiteRook = chessBoard[originalBlackRookPosition];
            chessBoard[originalBlackKingPosition] = '-';
            chessBoard[originalBlackRookPosition] ='-';
            chessBoard[newBlackKingPosition] = pieceWhiteKing;
            chessBoard[newBlackRookPosition] = pieceWhiteRook;

            castleRights[0][0] = false;

        }
        // Checking if the move is black castling queen side and it is still allowed, then castle
        else if(move.compare("c8d8") == 0 && castleRights[1][1]){
            int originalBlackKingPosition = 4;
            int originalBlackRookPosition = 0;
            int newBlackKingPosition = 2;
            int newBlackRookPosition = 3;

            char pieceWhiteKing = chessBoard[originalBlackKingPosition];
            char pieceWhiteRook = chessBoard[originalBlackRookPosition];
            chessBoard[originalBlackKingPosition] = '-';
            chessBoard[originalBlackRookPosition] ='-';
            chessBoard[newBlackKingPosition] = pieceWhiteKing;
            chessBoard[newBlackRookPosition] = pieceWhiteRook;

            castleRights[0][0] = false;
        }
        // Checking if white king side rook got moved and castleRights were still allowed then remove them.
        else if(move.rfind("h1",0) == 0 && castleRights[0][0]){
            castleRights[0][0] = false;
            updateNotCastle(move);
        }
        // Checking if white queen side rook got moved and castleRights were still allowed then remove them.
        else if(move.rfind("a1",0) == 0 && castleRights[0][1]){
            castleRights[0][1] = false;
            updateNotCastle(move);
        }
        // Checking if black king side rook got moved and castleRights were still allowed then remove them.
        else if(move.rfind("h8",0) == 0 && castleRights[1][0]){
            castleRights[1][0] = false;
            updateNotCastle(move);
        }
        // Checking if black queen side rook got moved and castleRights were still allowed then remove them.
        else if(move.rfind("h1",0) == 0 && castleRights[1][1]){
            castleRights[1][1] = false;
            updateNotCastle(move);
        }
        // Checking if white king moves and castleRights were still allowed then remove them.
        else if(move.rfind("e1",0) == 0 && (castleRights[0][0] || castleRights[0][1])){
            castleRights[0][0] = false;
            castleRights[0][1] = false;
            updateNotCastle(move);
        }
        // Checking if black king moves and castleRights were still allowed then remove them.
        else if(move.rfind("e8",0) == 0 && (castleRights[1][0] || castleRights[1][1])){
            castleRights[1][0] = false;
            castleRights[1][1] = false;
            updateNotCastle(move);
        }
        // Otherwise do normal movement
        else{
            updateNotCastle(move);
        }
        updateFENPosition();
        
    }


    void ChessEngine::updateNotCastle(string const &move){
        int rowMoveFrom = (int)move[0] - 97;
        int columnMoveFrom = 8 - (move[1] - '0');
        int rowMoveTo = (int)move[2] - 97;
        int columnMoveTo = 8 - (move[3] - '0');

        int arrayPositionFrom = (columnMoveFrom*8) + rowMoveFrom;
        int arrayPositionTo = (columnMoveTo*8) + rowMoveTo;

        char pieceToMove = chessBoard[arrayPositionFrom];
        chessBoard[arrayPositionFrom] = '-';
        chessBoard[arrayPositionTo] = pieceToMove;
    }


    void ChessEngine::updateFENPosition(){
        int counterEmpty = 0;
        char piece;
        currentFENPosition = "";

        for(int position = 0; position < sizeof(chessBoard); position++){
            piece = chessBoard[position];

            // CHeck if current position is a mulitple of 8, if yes then add FEN seperator
            if(position%8 == 0){
                currentFENPosition += "/"; 
            }

            // Check if we have an empty position then increase counter
            if(piece == '-' && counterEmpty >= 0){
                counterEmpty++;
            }
            //Checking if we have an empty position and we are at the end of a column then we need
            //to reset counter and print the so far read empty Strings number
            else if(piece == '-' && counterEmpty >= 0 && position%8 == 7){
                currentFENPosition += to_string(counterEmpty);
                counterEmpty = 0;
            }
            // Check if we have a piece again and counter greater 0 then add the counter to it
            else if(piece != '-' && counterEmpty > 0){
                currentFENPosition += to_string(counterEmpty);
                counterEmpty = 0;
            }
            else{
                currentFENPosition += piece;
            }
        }

        currentFENPosition += " ";

        // Adding whoose move turn it is
        if(wholeMoves % 2 == 1){
            currentFENPosition += "w";
        }
        else{
            currentFENPosition += "b";
        }

        currentFENPosition += " ";

        // Adding Castle right for white king side if still possible
        if(castleRights[0][0]) currentFENPosition += "K";
        // Adding Castle right for white queen side if still possible
        if(castleRights[0][1]) currentFENPosition += "Q";
        // Adding Castle right for black king side if still possible
        if(castleRights[1][0]) currentFENPosition += "k";
        // Adding Castle right for black queen side if still possible
        if(castleRights[1][1]) currentFENPosition += "q";

        currentFENPosition += " - 0 ";

        currentFENPosition += to_string(wholeMoves);
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

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    bool ChessEngine::getChessEngineStarted(){
        return chessEngineStarted;
    }

    // //////// //
    // Methods. //
    // //////// //
    void ChessEngine::playerMove(string &move){
        string chessEngineMove = "";

        if(dataChecker.isCorrectMove(move)){
            
            // Updating internal board state with player move
            updateBoardState(move);

            // Making the ChessEngine make the move.
            uciHandler.makeMove(currentFENPosition, searchOptions, chessEngineMove);

            // Updating internal board state with engine move
            updateBoardState(move);




        }
        // Checking if we got the move command to make the chessEngine move first.
        else if(move.compare("engineStart") == 0){
            // Making the ChessEngine make the move.
            uciHandler.makeMove(currentFENPosition, searchOptions, chessEngineMove);

            // Updating internal board state with engine move
            updateBoardState(move);
        }
        
        else{
            return;
        }
    }

    void ChessEngine::startNewGame(){
        if(!(uciHandler.startNewGame())) throw runtime_error("Couldn't start a fresh game!");
        wholeMoves = 0;
        castleRights[0][0] = true;
        castleRights[0][1] = true;
        castleRights[1][0] = true;
        castleRights[1][1] = true;
    }

    void ChessEngine::closeEngine(){
        uciHandler.closeProcess();
    }