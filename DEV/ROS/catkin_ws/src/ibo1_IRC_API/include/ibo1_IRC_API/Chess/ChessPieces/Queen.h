#ifndef QUEEN_H
#define QUEEN_H

#include <ibo1_IRC_API/Chess/ChessPieces/ChessPiece.h>
   
    // ////////// //
    // Constants. //
    // ////////// //

class Queen: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        Queen(bool isWhite){
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
            moveSets.push_back(UP_RIGHT);
            moveSets.push_back(UP_LEFT);
            moveSets.push_back(DOWN_RIGHT);
            moveSets.push_back(DOWN_LEFT);

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
#endif //QUEEN_H