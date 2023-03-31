#include "ibo1_IRC_API/Chess/ChessBoard.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //
    ChessBoard::ChessBoard(){
        castleRights[0][0] = true;
        castleRights[0][1] = true;
        castleRights[1][0] = true;
        castleRights[1][1] = true;

        whiteTurn = true;

        wholeMoves = 0;

        for(int i = 0; i < 64; i++){
            chessBoard.push_back(Cell(i));
        }

        // Setting white Pieces
        Rook rookWhite1(true);
        chessBoard.at(0).setChessPiece(&rookWhite1);
        Knight knightWhite1(true);
        chessBoard.at(1).setChessPiece(&knightWhite1);
        Bishop bishopWhite1(true);
        chessBoard.at(2).setChessPiece(&bishopWhite1);
        Queen queenWhite(true);
        chessBoard.at(3).setChessPiece(&queenWhite);
        King kingWhite(true);
        chessBoard.at(4).setChessPiece(&kingWhite);
        Bishop bishopWhite2(true);
        chessBoard.at(5).setChessPiece(&bishopWhite2);
        Knight knightWhite2(true);
        chessBoard.at(6).setChessPiece(&knightWhite2);
        Rook rookWhite2(true);
        chessBoard.at(7).setChessPiece(&rookWhite2);

        //Pawns
        Pawn pawnWhite1(true);
        chessBoard.at(8).setChessPiece(&pawnWhite1);
        Pawn pawnWhite2(true);
        chessBoard.at(9).setChessPiece(&pawnWhite2);
        Pawn pawnWhite3(true);
        chessBoard.at(10).setChessPiece(&pawnWhite3);
        Pawn pawnWhite4(true);
        chessBoard.at(11).setChessPiece(&pawnWhite4);
        Pawn pawnWhite5(true);
        chessBoard.at(12).setChessPiece(&pawnWhite5);
        Pawn pawnWhite6(true);
        chessBoard.at(13).setChessPiece(&pawnWhite6);
        Pawn pawnWhite7(true);
        chessBoard.at(14).setChessPiece(&pawnWhite7);
        Pawn pawnWhite8(true);
        chessBoard.at(15).setChessPiece(&pawnWhite8);

        // Setting black Pieces
        Rook rookBlack1(false);
        chessBoard.at(63).setChessPiece(&rookBlack1);
        Knight knightBlack1(false);
        chessBoard.at(62).setChessPiece(&knightBlack1);
        Bishop bishopBlack1(false);
        chessBoard.at(61).setChessPiece(&bishopBlack1);
        Queen queenBlack(false);
        chessBoard.at(60).setChessPiece(&queenBlack);
        King kingBlack(false);
        chessBoard.at(59).setChessPiece(&kingBlack);
        Bishop bishopBlack2(false);
        chessBoard.at(58).setChessPiece(&bishopBlack2);
        Knight knightBlack2(false);
        chessBoard.at(57).setChessPiece(&knightBlack2);
        Rook rookBlack2(false);
        chessBoard.at(56).setChessPiece(&rookBlack2);

        //Pawns
        Pawn pawnBlack1(false);
        chessBoard.at(55).setChessPiece(&pawnBlack1);
        Pawn pawnBlack2(false);
        chessBoard.at(54).setChessPiece(&pawnBlack2);
        Pawn pawnBlack3(false);
        chessBoard.at(53).setChessPiece(&pawnBlack3);
        Pawn pawnBlack4(false);
        chessBoard.at(52).setChessPiece(&pawnBlack4);
        Pawn pawnBlack5(false);
        chessBoard.at(51).setChessPiece(&pawnBlack5);
        Pawn pawnBlack6(false);
        chessBoard.at(50).setChessPiece(&pawnBlack6);
        Pawn pawnBlack7(false);
        chessBoard.at(49).setChessPiece(&pawnBlack7);
        Pawn pawnBlack8(false);
        chessBoard.at(48).setChessPiece(&pawnBlack8);

    }

    ChessBoard::ChessBoard(string const &FenPosition){
        //Convert fenString to chessBoard
    }

    // ////////////// //
    // Class methods. //
    // ////////////// //

    bool ChessBoard::pawnMoveValid(){

    }

    /*
        return values meaning:
             0 move is valid
            -1 not that colors turn
            -2 starting Cell is empty, thus no moveable piece
    */
    int ChessBoard::isMoveValid(int startPos, int endPos){

        Cell startCell = chessBoard.at(startPos);
        Cell endCell = chessBoard.at(endPos);

        // Check if it is that colors turn, if not return -1
        if(!(whiteTurn == startCell.getChessPieceColor())) return -1;

        // Checking if startCell is empty, if yes return -2
        if(!startCell.isOccupied()) return -2;

        //Getting startCell chessPiece information
        ChessPiece* chessPiece = startCell.getChessPiece();
        vector<MoveSet> moveSets = chessPiece->getMoveSet();

        int multiCellMoveCount;

        // Checking if the piece can only move one cell, if yes 
        if(chessPiece->getOnlyMovesOneCell()) multiCellMoveCount = 1;
        else multiCellMoveCount = 8;

        // Creating variables for stretched movements and has collided
        int stretchedMove = 0;
        bool hasCollided;


        //Loop each move in moveSet
        for(MoveSet ms : moveSets){
            
            //Reseting that it collided with a piece
            hasCollided = false;

            //Loop through all the cells it can move through
            for(int moveCount = 1; moveCount <= multiCellMoveCount; moveCount++){

                //If we already collided in this moveSet then just skip logic
                if(hasCollided) break;

                //Calculating stretchedMove
                stretchedMove = (ms * moveCount) + startCell.getPosition();


                // Checking if we are out of bounds
                if(stretchedMove < 0 || stretchedMove > 63 ) break;

                // Getting a reference to the looked at cell
                Cell tempCell = chessBoard.at(stretchedMove);

                
                // Checking if the cell is occupied
                if(tempCell.isOccupied()){

                    // Checking if the cell we move into is same color as ours. If yes then we have collided and cant move in that direction
                    if(tempCell.getChessPieceColor() == startCell.getChessPieceColor()){
                        hasCollided = true;
                        break;
                    }
                }

                // Checking if our move is looked at Cell is our expected Cell
                if(tempCell.getPosition() == endCell.getPosition()){

                    //Need to check if piece is a pawn if yes need to do extra checks

                    chessPiece->setHasMoved(true);

                    return 0;

                }
            }
        }
        
    }

    void ChessBoard::getVecPosFromMove(string const &move, int &startPos, int &endPos){

        int rowMoveFrom = (int)move[0] - 97;
        int columnMoveFrom = 8 - (move[1] - '0');
        int rowMoveTo = (int)move[2] - 97;
        int columnMoveTo = 8 - (move[3] - '0');

        startPos = (columnMoveFrom*8) + rowMoveFrom;
        endPos = (columnMoveTo*8) + rowMoveTo;
    }

    void ChessBoard::processMove(string const &move){

        int startPos = 0;
        int endPos = 0;

        getVecPosFromMove(move, startPos, endPos);

        int isMoveValidCode = isMoveValid(startPos, endPos);
        

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

    bool ChessBoard::move(string const &move){

        if(DataChecker::isCorrectMove(move)){
            processMove(move);
            return true;
        }
        else{
            return false;
        }

    }