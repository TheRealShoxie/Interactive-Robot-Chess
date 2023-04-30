#ifndef PAWN_H
#define PAWN_H

/*
 * Pawn - Pawn Chess Piece
 * <p>
 * This Class defines what a Pawn piece is. It defines its move set and its name.
 * It overrides functionality of the class ChessPiece found in ChessPiece.h
 * 
 * <p>
 * 3rd party code is used in this class. It is a C++ version with adaptions from github user: Stevoisiak.
 * Link to the original code: https://github.com/Stevoisiak/JavaFX-Online-Chess
 * 
 * This is an adaption from the Pawn.java file
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

class Pawn: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // Constructor for initializing a Pawn with its color
        Pawn(bool isWhite){
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

        // Method used to get if the Pawn can only move one cell at a time
        // Overrides the method from ChessPiece Class
        bool getOnlyMovesOneCell(){
            return true;
        }

        // Method used to get the move set of the Pawn
        // Overrides the method from the ChessPiece Class
        vector<MoveSet> getMoveSet(){
            vector<MoveSet> moveSets;

            // Checks if the Pawn is white
            if(chessPieceWhite){
                moveSets.push_back(UP);
                moveSets.push_back(UP_RIGHT);
                moveSets.push_back(UP_LEFT);

                // Checks if the pawn has already moved
                if(!getHasMoved()){
                    moveSets.push_back(DOUBLE_UP);
                }                 
            }

            // Otherwise Pawn is black
            else{
                moveSets.push_back(DOWN);
                moveSets.push_back(DOWN_RIGHT);
                moveSets.push_back(DOWN_LEFT);

                // Checks if the pawn has already moved
                if(!getHasMoved()){
                    moveSets.push_back(DOUBLE_DOWN);
                }
            }


            return moveSets;
        }

        // Method used to get the name of the Pawn
        // Overrides the moved from the ChessPiece Class
        char getName(){
            if(chessPieceWhite) return 'P';
            else return 'p';
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
#endif //PAWN_H