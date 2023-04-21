#ifndef MOVESET_H
#define MOVESET_H
   
    // ////////// //
    // Constants. //
    // ////////// //

class MoveSet{
    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        MoveSet(int xPos, int yPos){
            x = xPos;
            y = yPos;
        }

        // //////// //
        // Methods. //
        // //////// //

        int getCellPos(int multiple, int cellOffset){
            int newX = x*multiple;
            int newY = y*multiple;

            int offSetX = cellOffset%8;
            int offSetY = cellOffset/8;

            int actualX = newX+offSetX;
            int actualY = newY+offSetY;

            if(actualX < 0 || actualX > 7) return -1;
            if(actualY < 0 || actualY > 7) return -1;

            return actualX+actualY*8;
        }

        string toString() const{
            return "x:" +to_string(x) +"/" +"y:" +to_string(y);
        }

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //

        

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
        int x;
        int y;
};

const MoveSet UP(0,-1);
const MoveSet UP_LEFT(-1,-1);
const MoveSet UP_RIGHT(1,-1);
const MoveSet DOWN(0,1);
const MoveSet DOWN_LEFT(-1,1);
const MoveSet DOWN_RIGHT(1,1);
const MoveSet RIGHT(1,0);
const MoveSet LEFT(-1,0);

const MoveSet KNIGHT_UP_LEFT(-1,-2);
const MoveSet KNIGHT_UP_RIGHT(1,-2);
const MoveSet KNIGHT_DOWN_LEFT(-1,2);
const MoveSet KNIGHT_DOWN_RIGHT(1,2);

const MoveSet KNIGHT_LEFT_UP(-2,-1);
const MoveSet KNIGHT_RIGHT_UP(2,-1);
const MoveSet KNIGHT_LEFT_DOWN(-2,1);
const MoveSet KNIGHT_RIGHT_DOWN(2,1);

const MoveSet DOUBLE_UP(0,-2);
const MoveSet DOUBLE_DOWN(0,2);

const MoveSet CASTLE_KING_SIDE_KING(2,0);
const MoveSet CASTLE_KING_SIDE_ROOK(-2,0);
const MoveSet CASTLE_QUEEN_SIDE_KING(-2,0);
const MoveSet CASTLE_QUEEN_SIDE_ROOK(3,0);




#endif //MOVESET_H