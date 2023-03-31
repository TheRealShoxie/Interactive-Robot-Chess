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
            cellIsOccupied = false;
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
            if(chessPiece != NULL){
                return chessPiece->isWhite();
            }else{
                return NULL;
            }
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
            return cellIsOccupied;
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
        bool cellIsOccupied;
        ChessPiece* chessPiece;
        int position;
        
};
#endif //CELL_H