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
        bool move(string const &move);

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
        bool pawnMoveValid();
        int isMoveValid(int startPos, int endPos);
        void getVecPosFromMove(string const &move, int &startPos, int &endPos);
        void processMove(string const &move);
        
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //
        string currentFENPosition;
        vector<Cell> chessBoard;
        int wholeMoves;
        bool whiteTurn;
        bool castleRights[2][2];

    
};
#endif //CHESSBOARD_H