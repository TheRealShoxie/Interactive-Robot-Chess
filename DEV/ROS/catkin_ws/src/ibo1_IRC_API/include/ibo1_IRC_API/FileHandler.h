#ifndef FILEHANDLER_H
#define FILEHANDLER_H
   
#include <iostream>
#include <fstream>

#include "ibo1_IRC_API/User.h"

  
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

        void writeFile(string filepathName, string dataToBeWritten);
        string readFile(string filepathName);
        vector<User> readUsers(string filepathName);

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