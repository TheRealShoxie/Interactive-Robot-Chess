#ifndef CHESSBOARD_H
#define CHESSBOARD_H
   
   
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
        ChessBoard();
        ChessBoard(string const &FenPosition);

        // //////// //
        // Methods. //
        // //////// //
        int move(string &move);
        string toFENString();
        string toString();

        int kingCheck(int startPos, int endPos, bool colorToCheckWhite);

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //
        bool wasLastMoveCastleMove(){
            return lastMoveCastle;
        }

    private:

        // ////////////// //
        // Class methods. //
        // ////////////// //
        int pawnMoveValid(Cell &startCell, Cell &endCell, vector<Cell> &chessBoardToCheck);
        int castleMoveValid(Cell &startCell, Cell &endCell);
        //int kingCheck(int startPos, int endPos, bool colorToCheckWhite);
        void castle(string const &moveKing, string const &moveRook, int &startPos, int &endPos, int &isMoveValidCode);
        int isMoveValid(int startPos, int endPos);
        void getVecPosFromMove(string const &move, int &startPos, int &endPos);
        int processMove(string const &move);
        int processPromotionMove(string const &move);
        void fromFENString(string const &fenPos);
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