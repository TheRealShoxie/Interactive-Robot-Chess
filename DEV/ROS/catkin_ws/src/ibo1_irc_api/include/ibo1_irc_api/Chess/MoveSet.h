#ifndef MOVESET_H
#define MOVESET_H

/*
 * MoveSet - MoveSet class definition
 * <p>
 * Used for defining a move set. Move sets are 2d cell movements for chesspieces. It defines these move sets and 
 * has the possibility to convert to a cellPos for chessboards which use 1dimensional storage for cells
 * 
 * <p>
 * 3rd party code is used in this class. It is a C++ version with adaptions from github user: Stevoisiak.
 * Link to the original code: https://github.com/Stevoisiak/JavaFX-Online-Chess
 * 
 * This is an adaption from the Space.java file
 * This also adds additional functionality
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see ChessBoard.h
 * @see ChessPiece.h
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
   
    // ////////// //
    // Constants. //
    // ////////// //

class MoveSet{
    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // Constructor for creating a moveset
        MoveSet(int xPos, int yPos){
            x = xPos;
            y = yPos;
        }

        // //////// //
        // Methods. //
        // //////// //

        // Method returns the move for a 1dimensional container
        // The container must represent a chess board with a1 is 0 and h8 is 63
        int getCellPos(int multiple, int cellOffset){
            int newX = x*multiple;
            int newY = y*multiple;

            int offSetX = cellOffset%8;
            int offSetY = cellOffset/8;

            int actualX = newX+offSetX;
            int actualY = newY+offSetY;

            if(actualX < 0 || actualX > 7) return -1;
            if(actualY < 0 || actualY > 7) return -1;

            return actualX+actualY*8;
        }

        // Method returns a string representation of the move in x and y
        string toString() const{
            return "x:" +to_string(x) +"/" +"y:" +to_string(y);
        }

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // Method returns the cells to move in X
        int getXMovement(){
            return x;
        }

        // Method returns the cells to move in y
        int getYMovement(){
            return y;
        }

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
        int x;
        int y;
};



// Defines standard movesets
const MoveSet UP(0,-1);
const MoveSet UP_LEFT(-1,-1);
const MoveSet UP_RIGHT(1,-1);
const MoveSet DOWN(0,1);
const MoveSet DOWN_LEFT(-1,1);
const MoveSet DOWN_RIGHT(1,1);
const MoveSet RIGHT(1,0);
const MoveSet LEFT(-1,0);

const MoveSet KNIGHT_UP_LEFT(-1,-2);
const MoveSet KNIGHT_UP_RIGHT(1,-2);
const MoveSet KNIGHT_DOWN_LEFT(-1,2);
const MoveSet KNIGHT_DOWN_RIGHT(1,2);

const MoveSet KNIGHT_LEFT_UP(-2,-1);
const MoveSet KNIGHT_RIGHT_UP(2,-1);
const MoveSet KNIGHT_LEFT_DOWN(-2,1);
const MoveSet KNIGHT_RIGHT_DOWN(2,1);

const MoveSet DOUBLE_UP(0,-2);
const MoveSet DOUBLE_DOWN(0,2);

const MoveSet CASTLE_KING_SIDE_KING(2,0);
const MoveSet CASTLE_KING_SIDE_ROOK(-2,0);
const MoveSet CASTLE_QUEEN_SIDE_KING(-2,0);
const MoveSet CASTLE_QUEEN_SIDE_ROOK(3,0);




#endif //MOVESET_H