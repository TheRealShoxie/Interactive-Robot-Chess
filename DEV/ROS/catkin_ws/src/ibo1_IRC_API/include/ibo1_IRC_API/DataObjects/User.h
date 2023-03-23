#ifndef TEMPLATECLASS_H
#define TEMPLATECLASS_H

#include <iostream>
#include <vector>


using namespace std;
    // ////////// //
    // Constants. //
    // ////////// //

// Datatype
typedef std::uint8_t BYTE;

class User{
    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        User();
        User(string setUsername, string setPassword);
        User(string setUsername, string setPassword, bool setAdmin);
        User(vector<BYTE> protocolData);
        User(string loadString);
        // //////// //
        // Methods. //
        // //////// //
        string save();
        string toString();
        bool operator==(const User& usr) const;

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        void setUsername(string setUsername);
        void setPassword(string setPassword);
        void setAdmin(bool setAdmin);
        string getUsername();
        string getPassword();
        bool isAdmin();

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
        string username;
        string password;
        bool admin;
        
};
#endif //TEMPLATECLASS_H