#include <ros/ros.h>

#include "ibo1_IRC_API/Chess/ChessBoard.h"

#include <gtest/gtest.h>

#include "iostream"

using namespace std;


//Testing chessBoard creation standardPos
TEST(ChessBoard, creationStandardPos){
    ChessBoard chessBoard = ChessBoard();
    string fenPos = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard creation with fenString
TEST(ChessBoard, creationWithFenString){
    string fenPos = "r1bqkbnr/ppp2ppp/2n5/3pp3/2P5/3BPN2/PP1P1PPP/RNBQK2R w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard creation with fenString2
TEST(ChessBoard, creationWithFenString2){
    string fenPos = "r3kbnr/ppp2ppp/2n5/3p2q1/P1P5/1P2P1P1/5P1P/1bB2RK1 w kq - 0 13";
    ChessBoard chessBoard = ChessBoard(fenPos);

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard creation with fenString3
TEST(ChessBoard, creationWithFenString3){
    string fenPos = "r3kbnr/ppp2ppp/2n5/3p2q1/P1P5/1P1bPPP1/7P/2B2RK1 w kq - 1 14";
    ChessBoard chessBoard = ChessBoard(fenPos);

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard creation with fenString4
TEST(ChessBoard, creationWithFenString4){
    string fenPos = "r3kbnr/ppp2ppp/8/4n1q1/P1p5/1P1bPPP1/7P/2BR2K1 w kq - 2 16";
    ChessBoard chessBoard = ChessBoard(fenPos);

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard move from standard Position valid pawn move
TEST(ChessBoard, MoveValidFromStartPosValidPawnMove){
    ChessBoard chessBoard = ChessBoard();
    string movement = "A2A4";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from standard Position invalid pawn move diagonal
TEST(ChessBoard, MoveValidFromStartPosInValidPawnMoveDiagonal){
    ChessBoard chessBoard = ChessBoard();
    string movement = "A2B3";
    int expectedReturn = -2;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from standard Position invalid queen move piece in the way
TEST(ChessBoard, MoveValidFromStartPosInValidQueenMove){
    ChessBoard chessBoard = ChessBoard();
    string movement = "D1D2";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from standard Position invalid knight move
TEST(ChessBoard, MoveValidFromStartPosInValidKnightMove){
    ChessBoard chessBoard = ChessBoard();
    string movement = "B1B2";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from standard Position valid knight move
TEST(ChessBoard, MoveValidFromStartPosValidKnightMove){
    ChessBoard chessBoard = ChessBoard();
    string movement = "B1C3";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);
    ASSERT_EQ(returned, expectedReturn);
}


//Testing chessBoard move from FenPosition invalid pawn move double black in the way
TEST(ChessBoard, MoveValidFromFenPosInValidPawnMoveDouble){
    string fenPos = "rnbqkbnr/p1pppppp/8/P7/8/1p6/1PPPPPPP/RNBQKBNR w KQkq - 0 4";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "B2B4";
    int expectedReturn = -1;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid pawn move single black in the way
TEST(ChessBoard, MoveValidFromFenPosInValidPawnMoveSingle){
    string fenPos = "rnbqkbnr/p1pppppp/8/P7/8/1p6/1PPPPPPP/RNBQKBNR w KQkq - 0 4";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "B2B3";
    int expectedReturn = -1;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition valid pawn move diagonal
TEST(ChessBoard, MoveValidFromFenPosValidPawnMoveDiagonal){
    string fenPos = "rnbqkbnr/p1pppppp/8/P7/8/1p6/1PPPPPPP/RNBQKBNR w KQkq - 0 4";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C2B3";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid pawn move diagonal empty cell
TEST(ChessBoard, MoveValidFromFenPosInValidPawnMoveDiagonalEmpty){
    string fenPos = "rnbqkbnr/p1pppppp/8/P7/8/1p6/1PPPPPPP/RNBQKBNR w KQkq - 0 4";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C2D3";
    int expectedReturn = -2;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid pawn move diagonal same piece there
TEST(ChessBoard, MoveValidFromFenPosInValidPawnMoveDiagonalSameColorPiece){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C2D3";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid bishop move diagonal same piece there
TEST(ChessBoard, MoveValidFromFenPosInValidBishopMoveDiagonalSameColorPiece){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C1A3";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition valid bishop move diagonal same piece there
TEST(ChessBoard, MoveValidFromFenPosValidBishopMoveDiagonal){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C1F4";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid rook move up
TEST(ChessBoard, MoveValidFromFenPosInValidRookMoveUp){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "H1H6";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}
//Testing chessBoard move from FenPosition invalid rook2 move up
TEST(ChessBoard, MoveValidFromFenPosInValidRookMoveUp2){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "A1A5";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition valid rook move up
TEST(ChessBoard, MoveValidFromFenPosValidRookMoveUp){
    string fenPos = "rnbqkbnr/p2ppppp/2p5/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "A1A3";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition to Moved FenPosition
TEST(ChessBoard, MoveHasBeenMadeToNewFenPosition){
    string fenPosStart = "rnbqkbnr/p2ppppp/2p5/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    string fenPosExpected = "rnbqkbnr/p2ppppp/2p5/P7/8/Rp1P4/1PP1PPPP/1NBQKBNR b Kkq - 1 5";
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "A1A3";

    chessBoard.move(movement);
    string returnedFenPos = chessBoard.toFENString();

    ASSERT_EQ(returnedFenPos, fenPosExpected);
}

//Testing chessBoard move from FenPosition to Moved FenPosition
TEST(ChessBoard, MoveHasBeenMadeToNewFenPosition2){
    string fenPosStart = "rnbqkbnr/p2ppppp/2p5/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    string fenPosExpected = "rnbqkbnr/p2ppppp/2p5/P7/5B2/1p1P4/1PP1PPPP/RN1QKBNR b KQkq - 1 5";
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "C1F4";

    chessBoard.move(movement);
    string returnedFenPos = chessBoard.toFENString();

    ASSERT_EQ(returnedFenPos, fenPosExpected);
}


//Testing chessBoard castle white king side from fenPosition invalid
TEST(ChessBoard, CastleWhiteKingSideFromFenPositionInvalid){
    string fenPosStart = "rnbqkbnr/ppp1p1pp/3p4/5p2/4P3/5N2/PPPP1PPP/RNBQKB1R w KQkq f6 0 3";
    int expectedReturn = -5;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "e1g1";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard castle white king side from fenPosition valid
TEST(ChessBoard, CastleWhiteKingSideFromFenPositionValid){
    string fenPosStart = "rnbqkbnr/ppp3pp/3pp3/5p2/4P3/3B1N2/PPPP1PPP/RNBQK2R w KQkq - 0 4";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "e1g1";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard castle white king side from fenPosition invalid as no permission
TEST(ChessBoard, CastleWhiteKingSideFromFenPositionInValidMissingCastleRights){
    string fenPosStart = "rnbqkbnr/ppp4p/3pp3/5pp1/4P3/3B1N2/PPPP1PPP/RNBQK2R w kq - 0 6";
    int expectedReturn = -6;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "E1G1";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testChessBoard");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}
