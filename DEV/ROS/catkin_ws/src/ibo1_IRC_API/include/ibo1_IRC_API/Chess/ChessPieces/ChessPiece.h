#ifndef CHESSPIECE_H
#define CHESSPIECE_H

#include <ibo1_IRC_API/Chess/MoveSet.h>
#include <vector>

using namespace std;
   
    // ////////// //
    // Constants. //
    // ////////// //

class ChessPiece{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //
        void setHasMoved(bool chessPieceHasMoved);
        bool getHasMoved();

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //
        bool isWhite(){
            return chessPieceWhite;
        }
        
        bool getOnlyMovesOneCell(){
            return onlyMovesOneCell;
        }

        vector<MoveSet> getMoveSet(){
            vector<MoveSet> moveSets;
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

    protected:
        // ////////////// //
        // Class methods. //
        // ////////////// //
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //
        bool hasMoved;
        bool chessPieceWhite;
        bool onlyMovesOneCell;
};
#endif //CHESSPIECE_H