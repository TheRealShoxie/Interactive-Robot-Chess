#ifndef CHESSENGINES_H
#define CHESSENGINES_H

#include <ibo1_IRC_API/Chess/ChessEngine.h>

    // ////////// //
    // Constants. //
    // ////////// //

class TemplateClass{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //
        static void read(string filePathName);
        void save(string filePathName);

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
        vector<ChessEngineDefinitionStruct> availableChessEngines;

    
};
#endif //CHESSENGINES_H