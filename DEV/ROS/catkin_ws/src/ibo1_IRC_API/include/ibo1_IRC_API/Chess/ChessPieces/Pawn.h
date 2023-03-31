#ifndef PAWN_H
#define PAWN_H

#include <ibo1_IRC_API/Chess/ChessPieces/ChessPiece.h>

    // ////////// //
    // Constants. //
    // ////////// //

class Pawn: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
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
        bool getOnlyMovesOneCell(){
            return false;
        }
        vector<MoveSet> getMoveSet(){
            vector<MoveSet> moveSets;

            if(isWhite()){
                moveSets.push_back(UP);
                moveSets.push_back(UP_RIGHT);
                moveSets.push_back(UP_LEFT);

                if(!getHasMoved()){
                    moveSets.push_back(DOUBLE_UP);
                }

            }
            else{
                moveSets.push_back(DOWN);
                moveSets.push_back(DOWN_RIGHT);
                moveSets.push_back(DOWN_LEFT);

                if(!getHasMoved()){
                    moveSets.push_back(DOUBLE_DOWN);
                } 
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
#endif //PAWN_H