#ifndef FILEHANDLER_H
#define FILEHANDLER_H
   
#include <iostream>
#include <fstream>

#include "ibo1_irc_api/User/User.h"
#include "ibo1_irc_api/Chess/ChessEngine.h"

  
    // ////////// //
    // Constants. //
    // ////////// //

class FileHandler{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //

        static void writeFile(string const &filepathName, string const &dataToBeWritten);
        static string readFile(string const &filepathName);
        static vector<User> readUsers(string const &filepathName);
        static vector<ChessEngineDefinitionStruct> readChessEngines(string const &filePathName);

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

    
};
#endif //FILEHANDLER_H