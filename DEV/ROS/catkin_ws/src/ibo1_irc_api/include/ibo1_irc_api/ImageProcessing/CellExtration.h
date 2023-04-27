#ifndef CELLEXTRACTION_H
#define CELLEXTRACTION_H

using namespace std;


int pixelOffSet = 5;

/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////// //
    // Structs.   //
    // ////////// //

    // Struct used for creating a set of x and y coordinates
    struct Image2dPoints{
        int x = 0;
        int y = 0;

        inline bool operator==(const Image2dPoints& e) const{
            return (x == e.x && y == e.y);
        }

        inline bool operator<(const Image2dPoints& e) const{
            if(x == e.x){
                //if(y == e.y) return false;
                return y < e.y;
            }
            return x < e.x;
        }
    };

    // Struct used for creating the rectangle which represents the chessboard square
    struct ImageSquares{
        int x1 = 0;
        int y1 = 0;
        int x2 = 0;
        int y2 = 0;

        inline bool operator==(const ImageSquares& e) const{
            return (x1 == e.x1 && y1 == e.y1);
        }

        inline bool operator<(const ImageSquares& e) const{
            // Checking if the pixel is within a range of another then we sort by y
            if((x1-pixelOffSet <= e.x1 && x1+pixelOffSet >= e.x1)){
                return y1 < e.y1;
            }
            return x1 < e.x1;
        }
    };

    // Struct used to represent the chessPiece information taken from the depthSensor 
    struct ImageChessPieceDepth{
        int x = 0;
        int y = 0;
        float depth = 0;
        bool isOccupied = false;

        inline bool operator==(const ImageChessPieceDepth& e) const{
            return (x == e.x && y == e.y);
        }

    };

    //Struct used to represent the chessPiece information from the image
    struct ImageChessCell{
        ImageChessPieceDepth chessPieceDepth;
        bool isWhite = false;

        inline bool operator==(const ImageChessCell& e) const{
            return (chessPieceDepth == e.chessPieceDepth);
        }

    };
   
#endif //CELLEXTRACTION_H