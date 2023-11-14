#ifndef CHESSBOARD_H
#define CHESSBOARD_H


/*
 * ChessBoard - Class which represents a chess board
 * <p>
 * Used as a chessboard representation. It holds cells and enables the function for legal and illegal move checking.
 * It allows the construction of a chessboard in a default state or from a FEN string. So far it allows general legal and illegal move checking.
 * Promotion moves and checking if a king is in check. It does not function for check mate yet.
 * 
 * <p>
 * 3rd party code is used in this class. It is a C++ version with adaptions from github user: Stevoisiak.
 * Link to the original code: https://github.com/Stevoisiak/JavaFX-Online-Chess
 * This class represents the ChessBoard.java file
 * 
 * <p>
 * Following methods are a C++ adaptation and reworked functionality from the original work:
 *  - ChessBoard()
 *  - move()
 *  - pawnMoveValid()
 *  - isMoveValid()
 *  - processMove()
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see ChessEngine.h
 * @see Cell.h
 * @see ChessBoard.cpp
 * @see DataChecker.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

// Includes the DataChecker and Cell
#include <ibo1_irc_api/Utility/DataChecker.h>
#include <ibo1_irc_api/Chess/Cell.h>
   
    // ////////// //
    // Constants. //
    // ////////// //

class ChessBoard{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // Default constructor for initializing a ChessBoard
        ChessBoard();

        // Constructor for initializing a ChessBoard from a FEN string
        ChessBoard(string const &FenPosition);

        // //////// //
        // Methods. //
        // //////// //
        
        // Method to make a move
        /*
            return values meaning:
                 0 move is valid
                -1 Pawn collided straight move
                -2 Pawn collided diagonal move
                -3 starting Cell is empty, thus no moveable piece
                -4 not that colors turn
                -5 went through all moveSets and did not find a correct move thus move invalid or it was blocked by own color
                -6 King doesn't have castleRights King side
                -7 King doesn't have castleRights Queen side
                -8 Own king in check
                -9 Other King in check mate
                -10 Pawn not allowed to move there as only promotion move is allowed
                -11 The piece is not a pawn thus cannot be used for promotion
                -12 Pawn is not moving into endPosition and cannot be used for promotion
                -13 Invalid piece name to promote into
                -14 Inputted move does not correspond format
        */
        int move(string &move);

        // Method to return a FEN representation of the chess board
        string toFENString();

        // Method to return a 2d string representation of the chess board
        string toString();

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //

        // Method to check if the last move was a castle move
        bool wasLastMoveCastleMove(){
            return lastMoveCastle;
        }

    private:

        // ////////////// //
        // Class methods. //
        // ////////////// //

        // Method to check if the move is a pawn move and if it is a valid pawn move
        /*
            return values meaning:
                 0 move is valid or is not a pawn
                -1 Pawn collided straight move
                -2 Pawn collided diagonal move
        */
        int pawnMoveValid(Cell &startCell, Cell &endCell, vector<Cell> &chessBoardToCheck);


        // Method to check if the castle move is valid
        /*
            return values meaning:
                 0 move is valid or is not a king
                -6 King doesn't have castleRights King side
                -7 King doesn't have castleRights Queen side
        */
        int castleMoveValid(Cell &startCell, Cell &endCell);


        // Method to check if the king is in check/mate
        /*
            Currently checkMate not implemented
            return values meaning:
                 0 no king in check
                -1 King in check
                -2 King in checkMate
        */
        int kingCheck(int startPos, int endPos, bool colorToCheckWhite);


        // Method to perform castling
        void castle(string const &moveKing, string const &moveRook, int &startPos, int &endPos, int &isMoveValidCode);


        // Method to check if the move to be done is valid
        /*
            return values meaning:
                 0 move is valid
                -1 Pawn collided straight move
                -2 Pawn collided diagonal move
                -3 starting Cell is empty, thus no moveable piece
                -4 not that colors turn
                -5 went through all moveSets and did not find a correct move thus move invalid or it was blocked by own color
                -6 King doesn't have castleRights King side
                -7 King doesn't have castleRights Queen side
                -8 Own king in check
                -9 Other King in check mate
        */
        int isMoveValid(int startPos, int endPos);


        // Method to get the vector position from a chess move
        void getVecPosFromMove(string const &move, int &startPos, int &endPos);


        // Method to process a chess move
        /*
            return values meaning:
                 0 move is valid
                -1 Pawn collided straight move
                -2 Pawn collided diagonal move
                -3 starting Cell is empty, thus no moveable piece
                -4 not that colors turn
                -5 went through all moveSets and did not find a correct move thus move invalid or it was blocked by own color
                -6 King doesn't have castleRights King side
                -7 King doesn't have castleRights Queen side
                -8 Own king in check
                -9 Other King in check mate
                -10 Pawn not allowed to move there as only promotion move is allowed
        */
        int processMove(string const &move);


        // Method to process a promotion move
        /*
            return values meaning:
                 0 move is valid
                -1 Pawn collided straight move
                -2 Pawn collided diagonal move
                -3 starting Cell is empty, thus no moveable piece
                -4 not that colors turn
                -5 went through all moveSets and did not find a correct move thus move invalid or it was blocked by own color
                -6 King doesn't have castleRights King side
                -7 King doesn't have castleRights Queen side
                -8 Own king in check
                -11 The piece is not a pawn thus cannot be used for promotion
                -12 Pawn is not moving into endPosition and cannot be used for promotion
                -13 Invalid piece name to promote into
        */
        int processPromotionMove(string const &move);


        // Method to set the state of the chessboard from a FEN string
        void fromFENString(string const &fenPos);


        // Method to update the internal FEN representation of the chessboard
        void fenVariableUpdate(int const &startPos, int const &endPos);
        
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //
        vector<Cell> chessBoard;
        int wholeMoves;
        int halfMoves;
        bool previousMoveWasPawn;
        bool whiteTurn;
        /*
            * [0][0] = white king side
            * [0][1] = white queen side
            * [0][0] = black king side
            * [0][0] = black queen side
        */
        bool castleRights[2][2];
        bool whiteKingInCheck;
        bool blackKingInCheck;
        bool lastMoveCastle;

    
};
#endif //CHESSBOARD_H