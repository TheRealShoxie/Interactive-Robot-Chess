#ifndef CELL_H
#define CELL_H

#include <memory>

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
        Cell(int pos){
            chessPiece = NULL;
            position = pos;
        }

        // //////// //
        // Methods. //
        // //////// //

        std::shared_ptr<ChessPiece> releaseChessPiece(){
            std::shared_ptr<ChessPiece> tempChessPiece = chessPiece;
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

        void setChessPiece(ChessPiece *newPiece) {
            setChessPiece(std::shared_ptr<ChessPiece>(newPiece));
        }

        void setChessPiece(std::shared_ptr<ChessPiece> setChessPiece){
            chessPiece = setChessPiece;
        }

        std::shared_ptr<ChessPiece> getChessPiece(){
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
        std::shared_ptr<ChessPiece> chessPiece;
        int position;
        
};
#endif //CELL_H