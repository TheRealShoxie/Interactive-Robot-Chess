#ifndef ROOK_H
#define ROOK_H

/*
 * Rook - Rook Chess Piece
 * <p>
 * This Class defines what a Rook piece is. It defines its move set and its name.
 * It overrides functionality of the class ChessPiece found in ChessPiece.h
 * 
 * <p>
 * 3rd party code is used in this class. It is a C++ version with adaptions from github user: Stevoisiak.
 * Link to the original code: https://github.com/Stevoisiak/JavaFX-Online-Chess
 * 
 * This is an adaption from the Rook.java file
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

class Rook: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // Constructor for initializing a Rook with its color
        Rook(bool isWhite){
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

        // Method used to get if the Rook can only move one cell at a time
        // Overrides the method from ChessPiece Class
        bool getOnlyMovesOneCell(){
            return false;
        }

        // Method used to get the move set of the Rook
        // Overrides the method from the ChessPiece Class
        vector<MoveSet> getMoveSet(){
            vector<MoveSet> moveSets;

            moveSets.push_back(UP);
            moveSets.push_back(RIGHT);
            moveSets.push_back(LEFT);
            moveSets.push_back(DOWN);

            if(!hasMoved){
                moveSets.push_back(CASTLE_KING_SIDE_ROOK);
                moveSets.push_back(CASTLE_QUEEN_SIDE_ROOK);
            }

            return moveSets;
        }

        // Method used to get the name of the Rook
        // Overrides the moved from the ChessPiece Class
        char getName(){
            if(chessPieceWhite) return 'R';
            else return 'r';
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
#endif //ROOK_H