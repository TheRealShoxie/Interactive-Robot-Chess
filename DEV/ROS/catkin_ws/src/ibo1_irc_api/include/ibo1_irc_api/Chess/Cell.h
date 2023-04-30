#ifndef CELL_H
#define CELL_H

/*
 * Cell - Chess board cell class
 * <p>
 * Used for storing chess Pieces inside of it. Enables the setting and releasing of chessPieces
 * and getting their respective names and color. Stores internally its position in the chessboard
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

// include for the memory library
#include <memory>

// include for the possible ChessPieces it can hold and where they are defined
#include <ibo1_irc_api/Chess/ChessPieces/ChessPiece.h>
#include <ibo1_irc_api/Chess/ChessPieces/Rook.h>
#include <ibo1_irc_api/Chess/ChessPieces/Bishop.h>
#include <ibo1_irc_api/Chess/ChessPieces/Knight.h>
#include <ibo1_irc_api/Chess/ChessPieces/Queen.h>
#include <ibo1_irc_api/Chess/ChessPieces/King.h>
#include <ibo1_irc_api/Chess/ChessPieces/Pawn.h>

   
    // ////////// //
    // Constants. //
    // ////////// //

class Cell{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // Constructor and giving it its initial position
        Cell(int pos){
            chessPiece = NULL;
            position = pos;
        }

        // //////// //
        // Methods. //
        // //////// //

        // Method to remove a chess piece from the cell
        std::shared_ptr<ChessPiece> releaseChessPiece(){
            std::shared_ptr<ChessPiece> tempChessPiece = chessPiece;
            setChessPiece(NULL);
            return tempChessPiece;
        }

        // Method to get the color of the chess piece if it exists
        // Otherwise it returns NULL
        bool getChessPieceColor(){
            if(chessPiece != NULL) return chessPiece->isWhite();
            else return NULL;
        }

        // Method to get the name of the chess piece if it exists
        // Otherwise it returns '-'
        char getChessPieceName(){
            if(chessPiece != NULL) return chessPiece->getName();
            else return '-';
        }

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // Method used for setting a chess piece with  C++ default pointers
        void setChessPiece(ChessPiece *newPiece) {
            setChessPiece(std::shared_ptr<ChessPiece>(newPiece));
        }

        // Method used for setting a chess piece with shared_ptr pointers
        void setChessPiece(std::shared_ptr<ChessPiece> setChessPiece){
            chessPiece = setChessPiece;
        }

        // Method used for getting the chess piece of the cell
        std::shared_ptr<ChessPiece> getChessPiece(){
            return chessPiece;
        }

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //
        // Method used for checking if the cell holds a chess piece
        bool isOccupied(){
            if(chessPiece != NULL) return true;
            else return false;
        }

        // Method used for getting the position of the cell inside the chessboard
        int getPosition(){
            return position;
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
        std::shared_ptr<ChessPiece> chessPiece;
        int position;
        
};
#endif //CELL_H