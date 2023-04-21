#ifndef UCIHANDLER_H
#define UCIHANDLER_H

#include <ibo1_irc_api/Utility/SubProcessHandler.h>
#include <ibo1_irc_api/Utility/DataCreator.h>
   
    // ////////// //
    // Structs.   //
    // ////////// //

    // ////////// //
    // Constants. //
    // ////////// //

class UCIHandler{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        UCIHandler();
        UCIHandler(string const &processFilePathName);

        // //////// //
        // Methods. //
        // //////// //

        bool isEngineReady();
        bool startUCI(vector<EngineOption> &engineOptions);

        void makeMove(string const &fenPosition, string const &searchSettings, string &chessEngineMove);
        void setEngineOption(string const &optionName, string const &value);
        bool startNewGame();
        ~UCIHandler();

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

    
};
#endif //UCIHANDLER_H