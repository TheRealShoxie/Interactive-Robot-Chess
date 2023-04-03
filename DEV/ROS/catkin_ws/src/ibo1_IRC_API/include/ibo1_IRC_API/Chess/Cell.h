#ifndef CELL_H
#define CELL_H

#include <ibo1_IRC_API/Chess/ChessPieces/ChessPiece.h>
#include <ibo1_IRC_API/Chess/ChessPieces/Rook.h>
#include <ibo1_IRC_API/Chess/ChessPieces/Bishop.h>
#include <ibo1_IRC_API/Chess/ChessPieces/Knight.h>
#include <ibo1_IRC_API/Chess/ChessPieces/Queen.h>
#include <ibo1_IRC_API/Chess/ChessPieces/King.h>
#include <ibo1_IRC_API/Chess/ChessPieces/Pawn.h>

   
    // ////////// //
    // Constants. //
    // ////////// //

class Cell{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        Cell(int pos){
            chessPiece = NULL;
            position = pos;
        }

        // //////// //
        // Methods. //
        // //////// //

        ChessPiece* releaseChessPiece(){
            ChessPiece* tempChessPiece = chessPiece;
            setChessPiece(NULL);
            return tempChessPiece;
        }

        bool getChessPieceColor(){
            if(chessPiece != NULL) return chessPiece->isWhite();
            else return NULL;
        }

        char getChessPieceName(){
            if(chessPiece != NULL) return chessPiece->getName();
            else return '-';
        }

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        void setChessPiece(ChessPiece* setChessPiece){
            chessPiece = setChessPiece;
        }

        ChessPiece* getChessPiece(){
            return chessPiece;
        }

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //
        bool isOccupied(){
            if(chessPiece != NULL) return true;
            else return false;
        }

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
        ChessPiece* chessPiece;
        int position;
        
};
#endif //CELL_H