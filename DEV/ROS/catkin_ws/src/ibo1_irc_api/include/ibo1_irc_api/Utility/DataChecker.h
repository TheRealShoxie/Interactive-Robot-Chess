#ifndef DATACHECKER_H
#define DATACHECKER_H

/*
 * DataChecker
 * <p>
 * This file defines the DataChecker Class. This is a class with static methods for checking data used over the system.
 * Main functionality it currently provides is checking of correct MoveFormar for promotion and simple ChessMovement
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see ChessBoard.h
 * @see CreateTargetNode.cpp
 * @see RobotArmStateMachineNode.cpp
*/

    // ////////// //
    // Includes.  //
    // ////////// //
   
#include <string>
#include <regex>

using namespace std;
   
    // ////////// //
    // Constants. //
    // ////////// //

class DataChecker{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //

        // Method to check if a move is the correct format
        static bool isCorrectMoveFormat(string const &moveString){

            // Checking if the moveString is longer than 4 then return false
            if(moveString.length() > 4) return false;
            
            regex r("[a-h][1-8][a-h][1-8]");
            smatch match;

            // Checking if the move matches the regex for chess move
            if(regex_search(moveString,match, r)) return true;
            return false;
        }

        // Method to check if a promotion move is the correct format
        static bool isCorrectMoveFormatPromotion(string const &moveString){

            // Checking if the moveString is longer than 5 then return false
            if(moveString.length() > 5) return false;
            
            regex r("[a-h][1-8][a-h][1-8][a-z]");
            smatch match;

            // Checking if the move matches the regex for promotion chess move
            if(regex_search(moveString,match, r)) return true;
            return false;
        }

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
#endif //DATACHECKER_H