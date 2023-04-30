/*
 * Users - Class which represents Users
 * <p>
 * This file is the implementation of the class definition in Users.h
 * Please refer to Users.h for more information.
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see Users.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

#include "ibo1_irc_api/User/Users.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // Default constructor for creating users
    Users::Users(){}

    // Creating users from a filepath
    Users::Users(string filePathName){
        read(filePathName);
    }

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // Method for adding a user
    void Users::addUser(User addUser){
        users.push_back(addUser);
    }
    
    // Method for setting current user
    void Users::setCurrentUser(User currentlyUsedUser){
        currentUser = currentlyUsedUser;
    }

    // Method for getting current user
    User Users::getCurrentUser(){
        return currentUser;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    // Reading in users from a filepathname
    void Users::read(string filePathName){

        //Cycling through each user got from the filehandler
        for(User user : FileHandler::readUsers(filePathName)){
            users.push_back(user);
        }
    }

    // Saving users to a specified filepathname
    void Users::save(string filePathName){

        FileHandler::writeFile(filePathName, toString());
    }

    // Find a specific User throws runtime error if user cannot be found
    User Users::findUser(User userToBeFound){
        int userPosition = find(users.begin(), users.end(), userToBeFound) - users.begin();

        if(userPosition != users.end() - users.begin()) return users.at(userPosition);
        else{
            throw runtime_error("Cannot find the User!");
        } 

    }

    // String which represents users
    // To string for users.
    string Users::toString(){
        string outPutString = "<Users>\n";

        for(User user : users){
            outPutString += "    " +user.toString() +"\n";
        }

        outPutString += "<Users>";

        return outPutString;
    }