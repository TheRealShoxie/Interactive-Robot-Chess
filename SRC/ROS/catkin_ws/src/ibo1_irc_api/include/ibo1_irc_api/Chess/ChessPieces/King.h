#ifndef KING_H
#define KING_H

/*
 * King - King Chess Piece
 * <p>
 * This Class defines what a King piece is. It defines its move set and its name.
 * It overrides functionality of the class ChessPiece found in ChessPiece.h
 * 
 * <p>
 * 3rd party code is used in this class. It is a C++ version with adaptions from github user: Stevoisiak.
 * Link to the original code: https://github.com/Stevoisiak/JavaFX-Online-Chess
 * 
 * This is an adaption from the King.java file
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

class King: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        
        // Constructor for initializing a King with its color
        King(bool isWhite){
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

        // Method used to get if the King can only move one cell at a time
        // Overrides the method from ChessPiece Class
        bool getOnlyMovesOneCell(){
            return true;
        }

        // Method used to get the move set of the King
        // Overrides the method from the ChessPiece Class
        vector<MoveSet> getMoveSet(){
            vector<MoveSet> moveSets;

            moveSets.push_back(UP);
            moveSets.push_back(RIGHT);
            moveSets.push_back(LEFT);
            moveSets.push_back(DOWN);
            moveSets.push_back(UP_RIGHT);
            moveSets.push_back(UP_LEFT);
            moveSets.push_back(DOWN_LEFT);
            moveSets.push_back(DOWN_RIGHT);

            // If the King has not moves he is allowed to castle
            if(!hasMoved){
                moveSets.push_back(CASTLE_KING_SIDE_KING);
                moveSets.push_back(CASTLE_QUEEN_SIDE_KING);
            }

            return moveSets;
        }


        // Method used to get the name of the King
        // Overrides the moved from the ChessPiece Class
        char getName(){
            if(chessPieceWhite) return 'K';
            else return 'k';
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
#endif //KING_H