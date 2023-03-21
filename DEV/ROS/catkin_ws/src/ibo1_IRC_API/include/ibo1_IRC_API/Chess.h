#ifndef CHESS_H
#define CHESS_H

#include <ibo1_IRC_API/UCIHandler.h>
#include <ibo1_IRC_API/SubProcessHandler.h>
#include <bitset>


    // ////////// //
    // Constants. //
    // ////////// //

class Chess{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        Chess();
        Chess(string processFilePathName);

        // //////// //
        // Methods. //
        // //////// //
        Chess& operator= (Chess&&){ return *this; }

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
        bitset<2> currentChessBoard[8][8] = {
            {3, 3, 3, 3, 4, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3}
        };
        // Capital letters = white, small = black, move color = w/b, KQkq = can castle, last number how many full moves
        string fenBoardState = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";

};
#endif //CHESS_H