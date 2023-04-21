#ifndef BISHOP_H
#define BISHOP_H

#include <ibo1_irc_api/Chess/ChessPieces/ChessPiece.h>
   
    // ////////// //
    // Constants. //
    // ////////// //

class Bishop: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        Bishop(bool isWhite){
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
        bool getOnlyMovesOneCell(){
            return false;
        }

        vector<MoveSet> getMoveSet(){
            vector<MoveSet> moveSets;

            moveSets.push_back(UP_RIGHT);
            moveSets.push_back(UP_LEFT);
            moveSets.push_back(DOWN_RIGHT);
            moveSets.push_back(DOWN_LEFT);

            return moveSets;
        }

        char getName(){
            if(chessPieceWhite) return 'B';
            else return 'b';
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
#endif //BISHOP_H