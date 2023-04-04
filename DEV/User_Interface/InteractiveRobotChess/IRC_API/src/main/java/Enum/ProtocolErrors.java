package Enum;

public enum ProtocolErrors {
    User_DOES_NOTEXIST,
    UNEXPECTED_RETURN_CMD,
    UNRECOGNIZABLE_CMD,
    CHESSENGINE_NOT_FOUND,
    CHESSENGINE_NOT_STARTED,
    CHESSENGINE_NOT_RUNNING,
    CHESSENGINE_OPTION_DOES_NOTEXIST,
    CHESSENGINE_SEARCH_OPTION_DOES_NOTEXIST,
    CHESSENGINE_PAWN_COLLIDED_STRAIGHT,
    CHESSENGINE_PAWN_COLLIDED_DIAGONAL_OREMPTYCELLTHERE,
    CHESSENGINE_STARTING_CELL_EMPTY,
    CHESSENGINE_NOT_THAT_COLORS_TURN,
    CHESSENGINE_MOVE_INVALID_OR_BLOCKED_BY_OWN_COLOR,
    CHESSENGINE_CANNOT_CASTLE_KING_SIDE,
    CHESSENGINE_CANNOT_CASTLE_QUEEN_SIDE,
    CHESSENGINE_PIECE_TO_PROMOTE_NOT_PAWN,
    CHESSENGINE_OWN_KING_IN_CHECK,
    CHESSENGINE_OTHER_KING_IN_CHECK_MATE,
    CHESSENGINE_PAWN_NOT_MOVING_INTO_END_POS,
    CHESSENGINE_INVALID_NAME_FOR_PIECE_TO_PROMOTE_INTO,
    CHESSENGINE_PAWN_NOT_ALLOWED_NOT_PROMOTION_MOVE,
    CHESSENGINE_CREATED_NO_MOVE,
    CHESSENGINE_MOVE_FORMAT_INVALID

}
