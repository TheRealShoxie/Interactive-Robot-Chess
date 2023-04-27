#include <ros/ros.h>


// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Ros msg type includes
#include <opencv_apps/Point2DArrayStamped.h>
#include "std_msgs/String.h"

// ImageCellExtraction header include
#include <ibo1_irc_api/ImageProcessing/CellExtration.h>

// General c++ includes
#include <iostream>
#include <set>

// Including publisher msg Type
#include <ibo1_irc_api/ChessCells.h>



/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //

// Window Names to displayed for debug

static const std::string OPENCV_WINDOW_Marked_Cells = "ChessBoard Cells marked window";
static const int offSetCell = 0;


// static const variables

// Points to square variable
static const int xSquareMinDistance = 30;
static const int xSquareMaxDistance = 45;
static const int ySquareMinDistance = 30;
static const int ySquareMaxDistance = 45;
set<Image2dPoints> sortedPoints;
set<ImageSquares> imageSquares;


// Chess Piece variables
vector<ImageChessPieceDepth> imageChessPiecesDepths;
vector<ImageChessCell> imageChessCells;
static const float minimumChessPieceHeight = 0.022;
static const float emptyCellDepth = 0.95;
static const int colorChessPieceCuttOfPoint = 8388607;



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //


//Callback for corners
void cornersMessageReceived(const opencv_apps::Point2DArrayStamped msg){
    // Clearing vectors and the set
    imageSquares.clear();
    sortedPoints.clear();

    
    // Moving all points into a set to order them and possible duplicates
    for(auto cornerPoint : msg.points ){
        Image2dPoints im2DP;
        im2DP.x = cornerPoint.x;
        im2DP.y = cornerPoint.y;
        sortedPoints.insert(im2DP);
    }


    // Now we create all the top right and bottom right rectangle positions from the points

    // Creating 2 iterators for the Image2dPoints set
    set<Image2dPoints>::iterator itrSP;
    set<Image2dPoints>::iterator itrSP2;

    int currentX = 0;
    int currentY = 0;
    int nextX = 0;
    int nextY = 0;

    // Iterating through all elements of the set
    for(itrSP = sortedPoints.begin(); itrSP != sortedPoints.end(); itrSP++){
        
        // Assigning current x and y
        currentX = itrSP->x;
        currentY = itrSP->y;

        // Iterating through the rest of the points
        for(itrSP2 = next(itrSP,1); itrSP2 != sortedPoints.end(); itrSP2++){

            // Assign the nextX
            nextX = itrSP2->x;

            // Checking if the next X is within a certain distance
            if((nextX - currentX) <= xSquareMaxDistance && (nextX - currentX)  >= xSquareMinDistance){
                nextY = itrSP2->y;

                //If X was within the threshold check the same for y
                if((nextY - currentY) <= ySquareMaxDistance && (nextY - currentY) >= ySquareMinDistance){

                    // If y was within the threshold then create the imgSquare and push it to the vector.
                    // After break out of the loop as we found our pair
                    ImageSquares  imgSquare;
                    imgSquare.x1 = currentX;
                    imgSquare.y1 = currentY;
                    imgSquare.x2 = nextX;
                    imgSquare.y2 = nextY;
                    imageSquares.insert(imgSquare);
                    break;
                }
            }
        }
    }

}

// Callback for extracting Chessboard Cells
void imageCb(const sensor_msgs::ImageConstPtr& msg) {

    //Clearing the vector for the iamgeChessPieces
    imageChessCells.clear();


    //Getting a OpenCV version of the image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg,
        sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Creating variables for the loop
    int counterCellNumber = 0;
    int chessPieceX = 0;
    int chessPieceY = 0;
    int emptyCellCenterX = 0;
    int emptyCellCenterY = 0;
    int singleRGBValue = 0;
    ImageChessCell chessCell;
    
    // Iterating through each Square to get the color of the chessPiece if it exists
    for(auto square : imageSquares){

        chessPieceX = 0;
        chessPieceY = 0;

        // Highlighting the squares on the debug image for debugging
        cv::rectangle(cv_ptr->image, cv::Point(square.x1+offSetCell, square.y1+offSetCell), cv::Point(square.x2-offSetCell, square.y2-offSetCell), cv::Scalar(0,255,0), 1.5 );


        //Checking if our depths Pieces size is smaller than our squares then return
        if(imageChessPiecesDepths.size() < imageSquares.size()) return;

        // Checking if the cell is occupied. If not get the color and add the chessPiece to the vector for chessPieces
        if(imageChessPiecesDepths.at(counterCellNumber).isOccupied){
            // Get the x and y center position of the chessPiece
            chessPieceX = imageChessPiecesDepths.at(counterCellNumber).x;
            chessPieceY = imageChessPiecesDepths.at(counterCellNumber).y;

            //Getting the color at the chess piece pixel
            cv::Vec3b rgbValue = cv_ptr->image.at<cv::Vec3b>(chessPieceY,chessPieceX);

            //Drawing circle around found chessPieces
            cv::circle(cv_ptr->image, cv::Point(chessPieceX, chessPieceY), 3, CV_RGB(255,0,0));

            //Calculating single RGB value
            singleRGBValue = rgbValue[0];
            singleRGBValue = (singleRGBValue << 8) + rgbValue[1];
            singleRGBValue = (singleRGBValue << 8) + rgbValue[2];

            // Checking if singleRGB value smaller than cutoffpoint if yes then chesspiece color is black else white
            if(singleRGBValue < colorChessPieceCuttOfPoint) chessCell.isWhite = false;
            else chessCell.isWhite = true;
        }
        // Otherwise set isWhite automatically to false
        else{
            emptyCellCenterX = imageChessPiecesDepths.at(counterCellNumber).x;
            emptyCellCenterY = imageChessPiecesDepths.at(counterCellNumber).y;
            cv::circle(cv_ptr->image, cv::Point(emptyCellCenterX, emptyCellCenterY), 3, CV_RGB(0,0,255));
            chessCell.isWhite = false;
        }

        // Assigning the depthvalues to the chessPiece
        chessCell.chessPieceDepth = imageChessPiecesDepths.at(counterCellNumber);

        // Adding the chessPiece to the vector of chessPieces
        imageChessCells.push_back(chessCell);

        counterCellNumber++;
    }

    
    // Displaying the image with the marked squares
    cv::imshow(OPENCV_WINDOW_Marked_Cells, cv_ptr->image);

    cv::waitKey(3);
}



// Callback for getting the depth values and checking if the cell is empty or not
void imageCbDepth(const sensor_msgs::ImageConstPtr& msg) {

    //Clearing the vector for the iamgeChessPiecesDepths
    imageChessPiecesDepths.clear();

    //This function uses x and y coordinates from the contour moments
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg,
        sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Following iterates through all imageSquares to get information if the cell is empty or not.
    // If not empty sets the center X and Y pixel and sets the cell to isOccupied
    int totalXPixelValue = 0;
    int totalYPixelValue = 0;
    float totalDepthValue = 0;

    int numberOfPixelX = 0;
    int numberOfPixelY = 0;
    int numberOfDepthValues = 0;

    //Values for empty square
    int totalXPixelValueEmpty = 0;
    int totalYPixelValueEmpty = 0;
    float totalDepthValueEmpty = 0;

    int numberOfPixelXEmpty = 0;
    int numberOfPixelYEmpty = 0;
    int numberOfDepthValuesEmpty = 0;

    // Iterating through each selected Chessboard cell
    for(auto square : imageSquares){

        // Reseting the values for each square
        // Variables for square with chessPiece
        totalXPixelValue = 0;
        totalYPixelValue = 0;
        totalDepthValue = 0;

        numberOfPixelX = 0;
        numberOfPixelY = 0;
        numberOfDepthValues = 0;

        //Values for empty square
        totalXPixelValueEmpty = 0;
        totalYPixelValueEmpty = 0;
        totalDepthValueEmpty = 0;

        numberOfPixelXEmpty = 0;
        numberOfPixelYEmpty = 0;
        numberOfDepthValuesEmpty = 0;

        // Iterating through each pixel of the square
        for(int x = square.x1; x <= square.x2; x++){
            for(int y = square.y1; y <= square.y2; y++){

                // Getting the depth value at the pixel
                float depthAtPixel = cv_ptr->image.at<float>(y, x);

                // Checking if that depthAtPixel is part of a piece
                if(depthAtPixel < (emptyCellDepth - minimumChessPieceHeight)){
                    // Adding the Pixel values to their totals and increasing the number of pixels we added
                    totalXPixelValue = totalXPixelValue + x;
                    numberOfPixelX++;

                    totalYPixelValue = totalYPixelValue + y;
                    numberOfPixelY++;

                    // Adding the Depth value to the total and increasing the number of depthValues
                    totalDepthValue = totalDepthValue + depthAtPixel;
                    numberOfDepthValues++;
                }else{
                    // Adding the Pixel values to their totals and increasing the number of pixels we added
                    totalXPixelValueEmpty = totalXPixelValueEmpty + x;
                    numberOfPixelXEmpty++;

                    totalYPixelValueEmpty = totalYPixelValueEmpty + y;
                    numberOfPixelYEmpty++;

                    // Adding the Depth value to the total and increasing the number of depthValues
                    totalDepthValueEmpty = totalDepthValueEmpty + depthAtPixel;
                    numberOfDepthValuesEmpty++;
                }
            }
        }

        // Creating the imageChessPieceDepth object if we have a number of pixel and depthValues
        ImageChessPieceDepth imageChessPieceDepth;

        // DO we have depthValues for a chessPieve
        if(numberOfDepthValues != 0){
            // Calculating center of the piece in X,Y and adding it to the imageChessPieceDepth object
            imageChessPieceDepth.x = totalXPixelValue / numberOfPixelX;
            imageChessPieceDepth.y = totalYPixelValue / numberOfPixelY;

            //Calculating avarage depth of the piece and adding it to the imageCHessPieceDepth object
            imageChessPieceDepth.depth = totalDepthValue / numberOfDepthValues;

            //Indicating that the cell is occupied
            imageChessPieceDepth.isOccupied = true;
        }
        // Otherwise use the empty cells depth
        else if(numberOfDepthValuesEmpty != 0){
            // Calculating center of the piece in X,Y and adding it to the imageChessPieceDepth object
            imageChessPieceDepth.x = totalXPixelValueEmpty / numberOfPixelXEmpty;
            imageChessPieceDepth.y = totalYPixelValueEmpty / numberOfPixelYEmpty;

            //Calculating avarage depth of the piece and adding it to the imageCHessPieceDepth object
            imageChessPieceDepth.depth = totalDepthValueEmpty / numberOfDepthValuesEmpty;

            //Indicating that the cell is occupied
            imageChessPieceDepth.isOccupied = false;
        }

        // Adding it to the vector
        imageChessPiecesDepths.push_back(imageChessPieceDepth);
    }

}



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // //////////////////// //
    // Internal Functions.  //
    // //////////////////// //


    // Used to publish the extracted information
    void publishCellInformation(ros::Publisher const *cellDetection_pub){

        // If we do not have 64 cells then do not publish
        if(imageChessCells.size() != 64) return;

        // Creating the necesarry objects for the message to be send
        ibo1_irc_api::ChessCells sendChessCells;
        ibo1_irc_api::ChessCell chessCell;
        vector<ibo1_irc_api::ChessCell> chessCells;

        // Iterating through each chessPiece to set to the msg type
        for(auto &chessPiece : imageChessCells){
            chessCell.x = chessPiece.chessPieceDepth.x;
            chessCell.y = chessPiece.chessPieceDepth.y;
            chessCell.depth = chessPiece.chessPieceDepth.depth;
            chessCell.isOccupied = chessPiece.chessPieceDepth.isOccupied;
            chessCell.isWhite = chessPiece.isWhite;

            chessCells.push_back(chessCell);
        }

        // Assigning the chessCells to the ibo1_irc_api::ChessCells msg
        sendChessCells.chessCells = chessCells;

        // Publishing the msg
        cellDetection_pub->publish(sendChessCells);
        
    }



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "cellDetectionNode");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher cellDetection_pub = nh.advertise<ibo1_irc_api::ChessCells>("chessCellDetection", 10);
    ros::Subscriber corners_sub = nh.subscribe("/imageProcessing/goodfeature_filter/corners", 5, &cornersMessageReceived);

    image_transport::ImageTransport itImage(nh);
    image_transport::Subscriber color_sub = itImage.subscribe("/external_camera_overhead/ec_depth/image_raw", 1, imageCb);

    image_transport::ImageTransport itDepth(nh);
    image_transport::Subscriber depth_sub = itDepth.subscribe("/external_camera_overhead/ec_depth/depth/image_raw", 1, imageCbDepth);

    cv::namedWindow(OPENCV_WINDOW_Marked_Cells);

    ros::Rate rate(10);

    while(ros::ok()){

        // Publishing the extracted information
        publishCellInformation(&cellDetection_pub);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}