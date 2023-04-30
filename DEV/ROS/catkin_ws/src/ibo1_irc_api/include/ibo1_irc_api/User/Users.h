#ifndef USERS_H
#define USERS_H

/*
 * Users - Class which represents a chess board
 * <p>
 * This class defines a container for users. It enables functionality such as finding a user,
 * reading in users from a file or saving users to a file. Further enables the possibility to
 * set a current user to be used for logging in a user and having him represent the user that
 * uses the system.
 * 
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see IRCServerNode.cpp
 * @see IRCServerNodeHelper.h
 * @see User.h
 * @see FileHandler.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

   
#include "ibo1_irc_api/User/User.h"
#include "ibo1_irc_api/Utility/FileHandler.h"

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

        // Default constructor for creating users
        Users();

        // Creating users from a filepath
        Users(string filePathName);

        // //////// //
        // Methods. //
        // //////// //

        // Reading in users from a filepathname
        void read(string filePathName);

        // Saving users to a specified filepathname
        void save(string filePathName);

        // Find a specific User throws runtime error if user cannot be found
        User findUser(User userToBeFound);

        // String which represents users
        string toString();

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // Method for adding a user
        void addUser(User addUser);

        // Method for setting current user
        void setCurrentUser(User currentlyUsedUser);

        // Method for getting current user
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