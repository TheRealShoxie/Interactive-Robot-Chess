#ifndef PAWN_H
#define PAWN_H

#include <ibo1_irc_api/Chess/ChessPieces/ChessPiece.h>

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
            return true;
        }

        vector<MoveSet> getMoveSet(){
            vector<MoveSet> moveSets;

            if(chessPieceWhite){
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


        char getName(){
            if(chessPieceWhite) return 'P';
            else return 'p';
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