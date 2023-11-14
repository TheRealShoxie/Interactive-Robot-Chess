#ifndef CHESSPIECE_H
#define CHESSPIECE_H

/*
 * ChessPiece - General Chess Piece
 * <p>
 * Used as an abstract representation of a chess piece. Implements generalized functionality which are overriden by the
 * chess Pieces themselves. Please refer to @see to see which files override functionality of this class
 * 
 * <p>
 * 3rd party code is used in this class. It is a C++ version with adaptions from github user: Stevoisiak.
 * Link to the original code: https://github.com/Stevoisiak/JavaFX-Online-Chess
 * 
 * This is an adaption from the Piece.java file
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see Cell.h
 * @see MoveSet.h
 * @see Rook.h
 * @see Bishop.h
 * @see Knight.h
 * @see Queen.h
 * @see King.h
 * @see Pawn.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //


#include <ibo1_irc_api/Chess/MoveSet.h>
#include <vector>

using namespace std;
   
    // ////////// //
    // Constants. //
    // ////////// //

class ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // Method to set the has moved variable
        void setHasMoved(bool chessPieceHasMoved){
            hasMoved = chessPieceHasMoved;
        }

        // Method to get the has moved variable
        bool getHasMoved(){
            return hasMoved;
        }

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //

        // Method used to check if chess piece is white
        bool isWhite(){
            return chessPieceWhite;
        }
        
        // Method used to check if the chess piece only moves one cell
        // This function is overriden by the chess piece classes
        virtual bool getOnlyMovesOneCell(){
            return onlyMovesOneCell;
        }

        // Method used to get the MoveSet for the chessPiece
        // This function is overriden by the chess piece classes
        virtual vector<MoveSet> getMoveSet() = 0;

        // Method used to get the name of the chess piece
        // This function is overriden by the chess piece classes
        virtual char getName(){
            return '-';
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

    protected:
        // ////////////// //
        // Class methods. //
        // ////////////// //
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //
        bool hasMoved;
        bool chessPieceWhite;
        bool onlyMovesOneCell;
};
#endif //CHESSPIECE_H