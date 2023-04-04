#ifndef KNIGHT_H
#define KNIGHT_H
   
#include <ibo1_IRC_API/Chess/ChessPieces/ChessPiece.h>

    // ////////// //
    // Constants. //
    // ////////// //

class Knight: public ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        Knight(bool isWhite){
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

            moveSets.push_back(KNIGHT_UP_LEFT);
            moveSets.push_back(KNIGHT_UP_RIGHT);
            moveSets.push_back(KNIGHT_DOWN_LEFT);
            moveSets.push_back(KNIGHT_DOWN_RIGHT);

            moveSets.push_back(KNIGHT_LEFT_UP);
            moveSets.push_back(KNIGHT_RIGHT_UP);
            moveSets.push_back(KNIGHT_LEFT_DOWN);
            moveSets.push_back(KNIGHT_RIGHT_DOWN);

            return moveSets;
        }

        char getName(){
            if(chessPieceWhite) return 'N';
            else return 'n';
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
#endif //KNIGHT_H