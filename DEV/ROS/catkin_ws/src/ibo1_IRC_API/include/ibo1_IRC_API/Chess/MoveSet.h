#ifndef MOVESET_H
#define MOVESET_H
   
    // ////////// //
    // Constants. //
    // ////////// //


enum MoveSet{
    UP = -8,
    UP_LEFT = -9,
    UP_RIGHT = -7,
    DOWN = 8,
    DOWN_LEFT = 7,
    DOWN_RIGHT = 9,
    RIGHT = 1,
    LEFT = -1,

    KNIGHT_UP_LEFT = -17,
    KNIGHT_UP_RIGHT = -15,
    KNIGHT_DOWN_LEFT = 15,
    KNIGHT_DOWN_RIGHT = 17,

    KNIGHT_LEFT_UP = -10,
    KNIGHT_RIGT_UP = -6,
    KNIGHT_LEFT_DOWN = 6,
    KNIGHT_RIGHT_DOWN = 10,

    DOUBLE_UP = -16,
    DOUBLE_DOWN = 16,

    CASTLE_KING_SIDE_KING = 2,
    CASTLE_KING_SIDE_ROOK = -2,
    CASTLE_QUEEN_SIDE_KING = -2,
    CASTLE_QUEEN_SIDE_ROOK = 3

};
#endif //MOVESET_H