#ifndef CHESSPIECE_H
#define CHESSPIECE_H

#include <ibo1_irc_api/Chess/MoveSet.h>
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
        void setHasMoved(bool chessPieceHasMoved){
            hasMoved = chessPieceHasMoved;
        }

        bool getHasMoved(){
            return hasMoved;
        }

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //
        bool isWhite(){
            return chessPieceWhite;
        }
        
        virtual bool getOnlyMovesOneCell(){
            return onlyMovesOneCell;
        }

        virtual vector<MoveSet> getMoveSet() = 0;

        virtual char getName(){
            return '-';
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