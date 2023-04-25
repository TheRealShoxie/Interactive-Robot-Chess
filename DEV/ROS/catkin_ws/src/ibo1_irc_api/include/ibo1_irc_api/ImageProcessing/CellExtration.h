#ifndef CELLEXTRACTION_H
#define CELLEXTRACTION_H

using namespace std;
   

    // ////////// //
    // Structs.   //
    // ////////// //
    struct ImageChessBoardCell{
        int x = 0;
        int y = 0;
        int length = 0;
        int area = 0;

        inline bool operator==(const ImageChessBoardCell& e) const{
            return (x == e.x && y == e.y);
        }

        inline bool operator<(const ImageChessBoardCell& e) const{
            if(x == e.x){
                //if(y == e.y) return false;
                return y < e.y;
            }
            return x < e.x;
        }
    };

    struct ImageChessPiece{
        int x = 0;
        int y = 0;
        float depth = 0;
        bool isOccupied = false;
    };

    struct ImageChessPieceWithColor{
        ImageChessPiece imageChessPiece;
        bool isWhite = false;
    };
   
#endif //IMAGECELL_H