#ifndef USERS_H
#define USERS_H
   
#include "ibo1_IRC_API/DataObjects/User.h"
#include "ibo1_IRC_API/Utility/FileHandler.h"

#include <algorithm>
// Used for throwing exceptions
#include <stdexcept>

    // ////////// //
    // Constants. //
    // ////////// //

class Users{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        Users();
        Users(string filePathName);

        // //////// //
        // Methods. //
        // //////// //
        void read(string filePathName);
        void save(string filePathName);
        User findUser(User userToBeFound);
        string toString();

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        void addUser(User addUser);
        void setCurrentUser(User currentlyUsedUser);
        User getCurrentUser();

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
        std::vector<User> users;
        User currentUser;

    
};
#endif //USERS_H