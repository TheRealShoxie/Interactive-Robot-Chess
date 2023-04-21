#include "ibo1_irc_api/User/User.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //
    User::User(){
        username = "";
        password = "";
        admin = false;
    }

    User::User(string setUsername, string setPassword){
        username = setUsername;
        password = setPassword;
        admin = false;
    }

    User::User(string setUsername, string setPassword, bool setAdmin){
        username = setUsername;
        password = setPassword;
        admin = setAdmin;
    }

    User::User(vector<BYTE> protocolData){
        // Build with the help of: https://favtutor.com/blogs/split-string-cpp
        string dataString = string(protocolData.begin(), protocolData.end());

        string splitString = "\u241f";
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

    void User::setUsername(string setUsername){
        username = setUsername;
    }

    void User::setPassword(string setPassword){
        password = setPassword;
    }

    void User::setAdmin(bool setAdmin){
        admin = setAdmin;
    }

    string User::getUsername(){
        return username;
    }

    string User::getPassword(){
        return password;
    }

    bool User::isAdmin(){
        return admin;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //
    

    //Adds 4 spaces at the front of the toString(). Used for saving.
    string User::save(){
        return "    " + toString();
    }

    // toString for a user
    string User::toString(){
        return "<username=" + username + ";password=" + password + ";isAdmin=" + to_string(admin) + ">"; 
    }

    //Written using https://stackoverflow.com/questions/69792401/how-can-i-override-operator-to-compare-two-objects
    bool User::operator==(const User& usr) const{
        return  username == usr.username &&
                password == usr.password;
    }