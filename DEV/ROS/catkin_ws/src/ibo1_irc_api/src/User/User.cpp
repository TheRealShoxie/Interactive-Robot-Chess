/*
 * User - Class which defines a User
 * <p>
 * This file is the implementation of the class definition in User.h
 * Please refer to User.h for more information.
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see User.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

#include "ibo1_irc_api/User/User.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // Default constructor
    User::User(){
        username = "";
        password = "";
        admin = false;
    }


    // Constructor for setting a username and password. admin is defaulted to false
    User::User(string setUsername, string setPassword){
        username = setUsername;
        password = setPassword;
        admin = false;
    }

    // Constructor for setting username, password and isAdmin
    User::User(string setUsername, string setPassword, bool setAdmin){
        username = setUsername;
        password = setPassword;
        admin = setAdmin;
    }

    // Constructor for creating a based of protocolData
    User::User(vector<BYTE> protocolData){

        // Converts the data from the protocol to a string
        string dataString = string(protocolData.begin(), protocolData.end());

        // Setting the splitter for data defined by the protocol
        string splitString = "\u241f";

        // Finding the splitter and splitting the data up into username and password
        int end = dataString.find(splitString);


        username = dataString.substr(0, end);
        dataString.erase(dataString.begin(), dataString.begin() + end + splitString.size());
        password = dataString.substr(0, -1);
        admin = false;
    }


    // ////////////// //
    // Class methods. //
    // ////////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // Setting the username
    void User::setUsername(string setUsername){
        username = setUsername;
    }

    // Setting the password
    void User::setPassword(string setPassword){
        password = setPassword;
    }

    // Setting the admin bool
    void User::setAdmin(bool setAdmin){
        admin = setAdmin;
    }

    // Getting the username
    string User::getUsername(){
        return username;
    }

    // Getting the password
    string User::getPassword(){
        return password;
    }

    // Getting the admin bool
    bool User::isAdmin(){
        return admin;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //
    

    // Method that creates the save string for saving the user
    string User::save(){
        return "    " + toString();
    }

    // Method that creates a string representation of the user
    string User::toString(){
        return "<username=" + username + ";password=" + password + ";isAdmin=" + to_string(admin) + ">"; 
    }

    // Method used for comparing users
    bool User::operator==(const User& usr) const{
        return  username == usr.username &&
                password == usr.password;
    }