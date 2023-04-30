#ifndef KNIGHT_H
#define KNIGHT_H

/*
 * Knight - Knight Chess Piece
 * <p>
 * This Class defines what a Knight piece is. It defines its move set and its name.
 * It overrides functionality of the class ChessPiece found in ChessPiece.h
 * 
 * <p>
 * 3rd party code is used in this class. It is a C++ version with adaptions from github user: Stevoisiak.
 * Link to the original code: https://github.com/Stevoisiak/JavaFX-Online-Chess
 * 
 * This is an adaption from the Knight.java file
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

class Knight: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        // Constructor for initializing a Knight with its color
        Knight(bool isWhite){
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

        // Method used to get if the Knight can only move one cell at a time
        // Overrides the method from ChessPiece Class
        bool getOnlyMovesOneCell(){
            return true;
        }

        // Method used to get the move set of the Knight
        // Overrides the method from the ChessPiece Class
        vector<MoveSet> getMoveSet(){
            vector<MoveSet> moveSets;

            moveSets.push_back(KNIGHT_UP_LEFT);
            moveSets.push_back(KNIGHT_UP_RIGHT);
            moveSets.push_back(KNIGHT_DOWN_LEFT);
            moveSets.push_back(KNIGHT_DOWN_RIGHT);

            moveSets.push_back(KNIGHT_LEFT_UP);
            moveSets.push_back(KNIGHT_RIGHT_UP);
            moveSets.push_back(KNIGHT_LEFT_DOWN);
            moveSets.push_back(KNIGHT_RIGHT_DOWN);

            return moveSets;
        }

        // Method used to get the name of the Knight
        // Overrides the moved from the ChessPiece Class
        char getName(){
            if(chessPieceWhite) return 'N';
            else return 'n';
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
#endif //KNIGHT_H