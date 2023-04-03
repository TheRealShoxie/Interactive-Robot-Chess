#ifndef KING_H
#define KING_H
   
#include <ibo1_IRC_API/Chess/ChessPieces/ChessPiece.h>

    // ////////// //
    // Constants. //
    // ////////// //

class King: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
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
        bool getOnlyMovesOneCell(){
            return true;
        }
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

            if(!hasMoved){
                moveSets.push_back(CASTLE_KING_SIDE_KING);
                moveSets.push_back(CASTLE_QUEEN_SIDE_KING);
            }

            return moveSets;
        }

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