#ifndef CHESSENGINE_H
#define CHESSENGINE_H

#include <bitset>
#include <bits/stdc++.h> 


#include <ibo1_IRC_API/Utility/UCIHandler.h>
#include <ibo1_IRC_API/Chess/ChessBoard.h>
#include <ibo1_IRC_API/ProtocolAPI/ProtocolDefinition.h>

    // ////////// //
    // Structs.   //
    // ////////// //
    struct ChessEngineDefinitionStruct{
        string name = "";
        string filePathName = "";

        inline bool operator==(const ChessEngineDefinitionStruct& e) const{
            return (name.compare(e.name) == 0 && filePathName.compare(e.filePathName) == 0);
        }
    };


    // ////////// //
    // Constants. //
    // ////////// //

class ChessEngine{
    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        ChessEngine(string const &processFilePathName);

        // //////// //
        // Methods. //
        // //////// //
        void playerMove(BYTE &returnedProtocolByte, string &move);
        void chessEngineMove(BYTE &returnedProtocolByte, string &chessEngineMove);
        void setChessEngineOption(string const &optionName, string const &value);
        void startNewGame();
        string getChessBoardFENString();
        string getChessBoardString();
        


        ChessEngine& operator= (ChessEngine&&){ return *this; }

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //
        void setSearchOptions(string const &setSearchOptions);
        void getSearchOptions(string &getSearchOptions);
        void getChessEngineOptions(vector<EngineOption> &engineOptions);

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //
        bool getChessEngineStarted();

    private:

        // ////////////// //
        // Class methods. //
        // ////////////// //
        void getProtocolCode(int moveCode, BYTE const &defaultValue, BYTE &returnByte);
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //
        UCIHandler uciHandler;

        ChessBoard chessBoard;

        string searchOptions = "";
        vector<EngineOption> engineOptions;
        bool chessEngineStarted = false;

};
#endif //CHESSENGINE_H