#include "ibo1_IRC_API/User/Users.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    Users::Users(){}
    Users::Users(string filePathName){
        read(filePathName);
    }

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //


    void Users::addUser(User addUser){
        users.push_back(addUser);
    }
    
    void Users::setCurrentUser(User currentlyUsedUser){
        currentUser = currentlyUsedUser;
    }

    User Users::getCurrentUser(){
        return currentUser;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    // Reads in all users from a file.
    void Users::read(string filePathName){

        for(User user : FileHandler::readUsers(filePathName)){
            users.push_back(user);
        }

    }

    // Saves all users from a file
    // Written using https://www.w3schools.com/cpp/cpp_files.asp
    void Users::save(string filePathName){

        FileHandler::writeFile(filePathName, toString());
    }

    User Users::findUser(User userToBeFound){
        int userPosition = find(users.begin(), users.end(), userToBeFound) - users.begin();

        if(userPosition != users.end() - users.begin()) return users.at(userPosition);
        else{
            throw runtime_error("Cannot find the User!");
        } 

    }

    // To string for users.
    string Users::toString(){
        string outPutString = "<Users>\n";

        for(User user : users){
            outPutString += "    " +user.toString() +"\n";
        }

        outPutString += "<Users>";

        return outPutString;
    }