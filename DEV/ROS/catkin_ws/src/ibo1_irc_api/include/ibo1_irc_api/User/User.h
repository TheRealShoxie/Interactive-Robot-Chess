#ifndef TEMPLATECLASS_H
#define TEMPLATECLASS_H


/*
 * User - Class which represents a user
 * <p>
 * This class defines a user class. It defines what a user is made out of.
 * It enables the functionality to create a user frm multiple different inputs.
 * 
 * This is used for User creation and logging in on the system.
 * A Admin boolean allows the system to understand what functionalities
 * a user should be allowed to use.
 * 
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see Users.h
 * @see FileHandler.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

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

        // Default constructor
        User();

        // Constructor for setting a username and password. admin is defaulted to false
        User(string setUsername, string setPassword);

        // Constructor for setting username, password and isAdmin
        User(string setUsername, string setPassword, bool setAdmin);

        // Constructor for creating a based of protocolData, isAdmin defaulted to false
        User(vector<BYTE> protocolData);
        // //////// //
        // Methods. //
        // //////// //

        // Method that creates the save string for saving the user
        string save();

        // Method that creates a string representation of the user
        string toString();

        // Method used for comparing users
        bool operator==(const User& usr) const;

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // Setting the username
        void setUsername(string setUsername);

        // Setting the password
        void setPassword(string setPassword);

        // Setting the admin bool
        void setAdmin(bool setAdmin);

        // Getting the username
        string getUsername();

        // Getting the password
        string getPassword();

        // Getting the admin bool
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