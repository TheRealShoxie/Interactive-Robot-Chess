#ifndef CHESSBOARD_H
#define CHESSBOARD_H
   
   
#include <ibo1_IRC_API/Utility/DataChecker.h>
#include <ibo1_IRC_API/Chess/Cell.h>
   
    // ////////// //
    // Constants. //
    // ////////// //

class ChessBoard{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        ChessBoard();
        ChessBoard(string const &FenPosition);

        // //////// //
        // Methods. //
        // //////// //
        int move(string &move);
        string toFENString();
        string toString();

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
        int pawnMoveValid(Cell startCell, Cell endCell);
        int castleMoveValid(Cell startCell, Cell endCell);
        void castle(string const &moveKing, string const &moveRook, int &startPos, int &endPos, int &isMoveValidCode);
        int isMoveValid(int startPos, int endPos);
        void getVecPosFromMove(string const &move, int &startPos, int &endPos);
        int processMove(string const &move);;
        void fromFENString(string const &fenPos);
        
        
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

    
};
#endif //CHESSBOARD_H