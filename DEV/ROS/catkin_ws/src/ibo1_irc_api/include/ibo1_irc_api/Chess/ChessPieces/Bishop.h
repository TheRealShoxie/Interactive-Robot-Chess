#ifndef BISHOP_H
#define BISHOP_H

/*
 * Bishop - Bishop Chess Piece
 * <p>
 * This Class defines what a Bishop piece is. It defines its move set and its name.
 * It overrides functionality of the class ChessPiece found in ChessPiece.h
 * 
 * <p>
 * 3rd party code is used in this class. It is a C++ version with adaptions from github user: Stevoisiak.
 * Link to the original code: https://github.com/Stevoisiak/JavaFX-Online-Chess
 * 
 * This is an adaption from the Bishop.java file
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see Cell.h
 * @see MoveSet.h
 * @see ChessPiece.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

#include <ibo1_irc_api/Chess/ChessPieces/ChessPiece.h>
   
    // ////////// //
    // Constants. //
    // ////////// //

class Bishop: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        
        // Constructor for initializing a Bishop with its color
        Bishop(bool isWhite){
            chessPieceWhite = isWhite;
        }

        // //////// //
        // Methods. //
        // //////// //

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //

        // Method used to get if the Bishop can only move one cell at a time
        // Overrides the method from ChessPiece Class
        bool getOnlyMovesOneCell(){
            return false;
        }

        // Method used to get the move set of the Bishop
        // Overrides the method from the ChessPiece Class
        vector<MoveSet> getMoveSet(){
            vector<MoveSet> moveSets;

            moveSets.push_back(UP_RIGHT);
            moveSets.push_back(UP_LEFT);
            moveSets.push_back(DOWN_RIGHT);
            moveSets.push_back(DOWN_LEFT);

            return moveSets;
        }

        // Method used to get the name of the Bishop
        // Overrides the moved from the ChessPiece Class
        char getName(){
            if(chessPieceWhite) return 'B';
            else return 'b';
        }

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
#endif //BISHOP_H