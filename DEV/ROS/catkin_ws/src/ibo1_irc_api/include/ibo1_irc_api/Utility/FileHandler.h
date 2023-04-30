#ifndef FILEHANDLER_H
#define FILEHANDLER_H

/*
 * FileHandler
 * <p>
 * FileHandler CLass is used to interact with system Files.
 * 
 * The main implementation in the current system is the read Users and chess engines.
 * 
 * 
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see Users.h
 * @see ChessWrapperNode.cpp
 * @see User.h
 * @see DataCreator.h
*/
   
#include <iostream>
#include <fstream>

#include "ibo1_irc_api/User/User.h"
#include "ibo1_irc_api/Utility/DataCreator.h"

  
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

        // Method to write a string of data to a supplied filepath
        // throws invalid_argument if the file does not exist or cannot be found
        static void writeFile(string const &filepathName, string const &dataToBeWritten);

        // Method to read a file at a specified filePath
        // throws invalid_argument if the file does not exist or cannot be found
        static string readFile(string const &filepathName);

        // Method to readUsers from a supplied filepath
        // throws invalid_argument if the file does not exist or cannot be found
        static vector<User> readUsers(string const &filepathName);

        // Method to read chessEngines from a specified filepath
        // throws invalid_argument if the file does not exist or cannot be found
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