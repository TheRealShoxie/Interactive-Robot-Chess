#ifndef CHESS_H
#define CHESS_H

#include <ibo1_IRC_API/UCIHandler.h>
#include <bitset>

    // ////////// //
    // Structs.   //
    // ////////// //


    // ////////// //
    // Constants. //
    // ////////// //

class Chess{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        Chess();
        Chess(string const &processFilePathName);

        // //////// //
        // Methods. //
        // //////// //
        void playerMove(string &move);




        Chess& operator= (Chess&&){ return *this; }

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //
        void setSearchOptions(string const &setSearchOptions);
        void getSearchOptions(string &getSearchOptions);

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
        bitset<2> currentChessBoard[8][8] = {
            {3, 3, 3, 3, 4, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3}
        };
        // Capital letters = white, small = black, move color = w/b, KQkq = can castle, last number how many full moves
        string fenBoardState;
        UCIHandler uciHandler;
        string searchOptions;
        vector<EngineOption> engineOptions;
        

};
#endif //CHESS_H