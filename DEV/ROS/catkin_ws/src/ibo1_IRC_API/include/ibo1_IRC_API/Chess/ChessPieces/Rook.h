#ifndef ROOK_H
#define ROOK_H

#include <ibo1_IRC_API/Chess/ChessPieces/ChessPiece.h>
   
    // ////////// //
    // Constants. //
    // ////////// //

class Rook: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        Rook(bool isWhite){
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

            moveSets.push_back(UP);
            moveSets.push_back(RIGHT);
            moveSets.push_back(LEFT);
            moveSets.push_back(DOWN);

            if(!getHasMoved()){
                moveSets.push_back(CASTLE_KING_SIDE_ROOK);
                moveSets.push_back(CASTLE_QUEEN_SIDE_ROOK);
            }

            return moveSets;
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
#endif //ROOK_H