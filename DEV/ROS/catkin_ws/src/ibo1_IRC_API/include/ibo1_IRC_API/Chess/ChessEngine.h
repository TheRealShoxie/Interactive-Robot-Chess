#ifndef CHESSENGINE_H
#define CHESSENGINE_H

#include <bitset>
#include <bits/stdc++.h> 


#include <ibo1_IRC_API/Utility/UCIHandler.h>
#include <ibo1_IRC_API/Utility/DataChecker.h>
    // ////////// //
    // Structs.   //
    // ////////// //


    // ////////// //
    // Constants. //
    // ////////// //

class ChessEngine{
    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        ChessEngine();
        ChessEngine(string const &processFilePathName);

        // //////// //
        // Methods. //
        // //////// //
        void playerMove(string &move);
        void startNewGame();


        ChessEngine& operator= (ChessEngine&&){ return *this; }

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
        void updateBoardState(string const &move);
        void updateFENPosition();
        void updateNotCastle(string const &move);
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //
        UCIHandler uciHandler;
        DataChecker dataChecker;
        /*
            * Might not be needed
        */
        // bitset<2> currentChessBoardBitSet[8][8] = {
        //     {3, 3, 3, 3, 4, 3, 3, 3},
        //     {3, 3, 3, 3, 3, 3, 3, 3},
        //     {3, 3, 3, 3, 3, 3, 3, 3},
        //     {3, 3, 3, 3, 3, 3, 3, 3},
        //     {3, 3, 3, 3, 3, 3, 3, 3},
        //     {3, 3, 3, 3, 3, 3, 3, 3},
        //     {3, 3, 3, 3, 3, 3, 3, 3},
        //     {3, 3, 3, 3, 3, 3, 3, 3}
        // };
        char chessBoard[64] = {
            'r','n','b','q','k','b','n','r',
            'p','p','p','p','p','p','p','p',
            '-','-','-','-','-','-','-','-',
            '-','-','-','-','-','-','-','-',
            '-','-','-','-','-','-','-','-',
            '-','-','-','-','-','-','-','-',
            'P','P','P','P','P','P','P','P',
            'R','N','B','Q','K','B','N','R'
        };
        // Capital letters = white, small = black, move color = w/b, KQkq = can castle, last number how many full moves
        string currentFENPosition;
        int wholeMoves;
        char colorTurn;
        /*
            * [0][0] = white king side
            * [0][1] = white queen side
            * [0][0] = black king side
            * [0][0] = black queen side

        */
        bool castleRights[2][2] = {
            {true, true},
            {true, true}
        };


        string searchOptions;
        vector<EngineOption> engineOptions;
        

};
#endif //CHESSENGINE_H