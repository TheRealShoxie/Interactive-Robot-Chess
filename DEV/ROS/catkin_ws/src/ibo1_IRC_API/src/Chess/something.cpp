int isMoveValid(int startPos, int endPos){


        Cell &startCell = chessBoard.at(startPos);
        Cell &endCell = chessBoard.at(endPos);


        // Checking if startCell is empty, if yes return -3
        if(!startCell.isOccupied()) return -3;

        // Check if it is that colors turn, if not return -4
        if(!(whiteTurn == startCell.getChessPieceColor())) return -4;

        //Getting startCell chessPiece information
        std::shared_ptr<ChessPiece> chessPiece = startCell.getChessPiece();
        vector<MoveSet> moveSets = chessPiece->getMoveSet();

        int multiCellMoveCount;

        // Checking if the piece can only move one cell, if yes 
        if(chessPiece->getOnlyMovesOneCell()) multiCellMoveCount = 1;
        else multiCellMoveCount = 8;

        // Creating variables for stretched movements and has collided
        int stretchedMove = 0;
        bool hasCollided;


        //Loop each move in moveSet
        for(MoveSet ms : moveSets){
            
            //Resetting that we have collided as we are looking through a new move set
            hasCollided = false;

            //Loop through all the cells it can move through
            for(int moveCount = 1; moveCount <= multiCellMoveCount; moveCount++){

                //If we already collided in this moveSet then just skip logic
                if(hasCollided) break;

                //Calculating stretchedMove
                stretchedMove = (ms * moveCount) + startCell.getPosition();

                // Checking if we are out of bounds
                if(stretchedMove < 0 || stretchedMove > 63 ) break;

                // Getting a reference to the looked at cell
                Cell tempCell = chessBoard.at(stretchedMove);

                // Checking if the cell is occupied
                if(tempCell.isOccupied()){

                    // Checking if the cell we move through is same color as ours. If yes then we have collided and cant move in that direction
                    if(tempCell.getChessPieceColor() == startCell.getChessPieceColor()){
                        hasCollided = true;
                        break;
                    }

                    // Checking if the cell we move through is not same as color and we are not at the wanted reached position. If yes we have collided
                    if(tempCell.getChessPieceColor() != startCell.getChessPieceColor() && tempCell.getPosition() != endCell.getPosition()){
                        hasCollided = true;
                        break;
                    }
                }


                // Checking if our move is looked at Cell is our expected Cell
                if(tempCell.getPosition() == endCell.getPosition()){
                    
                    // Checking if pawnMoveValid
                    int returnedPawnMoveValid = pawnMoveValid(startCell, endCell);

                    // If we do not have a valid pawn move then return the error code
                    if(returnedPawnMoveValid != 0) return returnedPawnMoveValid;

                    int returnedCastleMoveValid = castleMoveValid(startCell, endCell);

                    if(returnedCastleMoveValid != 0) return returnedCastleMoveValid; 

                    //Otherwise it is correct move or not a pawn or correct move or not a king
                    //Need to check if piece is a pawn if yes need to do extra checks
                    chessPiece->setHasMoved(true);

                    return 0;
                }
            }
        }

        return -5;
        
    }