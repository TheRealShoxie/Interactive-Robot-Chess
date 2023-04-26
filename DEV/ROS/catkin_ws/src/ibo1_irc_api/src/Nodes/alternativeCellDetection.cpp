#include <ros/ros.h>

#include "std_msgs/String.h"

// OpenCV includes
#include <opencv_apps/MomentArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ImageCell header include
#include <ibo1_irc_api/ImageProcessing/CellExtration.h>

// General c++ imports
#include <iostream>
#include <set>

// Test
#include <fstream>


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
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


static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW_Cells = "ChessBoard Cell window";

static const int x_max = 1280;
static const int y_max = 720;
static const int x_max_half = x_max/2;
static const int y_max_half = y_max/2;

static const int offSetCenter = 0;
static const float minimumPieceHeight = 0.02;
static const float emptyCellDepth = 0.95;

vector<ImageChessPieceWithColor> imageChessPiecesWithColors;

vector<opencv_apps::Moment> notChosenMoments;
set<ImageChessBoardCell> chosenImageChessBoardCellS;
static const int lowThresholdArea = 800;
static const int highThresholdArea = 1050;

static const int colorChessPieceCuttOfPoint = 8388607;





/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

//Callback for contour moments
void momentsMessageReceived(const opencv_apps::MomentArrayStamped msg){
    int numberMoment = 0;
    int numberOfActualCells = 0;
    int numberOfNotChosenCells = 0;
    notChosenMoments.clear();
    chosenImageChessBoardCellS.clear();


    for(auto &moment : msg.moments){
        if(isnan(moment.center.x) || isnan(moment.center.y)) break;


        if(moment.area > lowThresholdArea && moment.area < highThresholdArea){
            numberOfActualCells++;

            ImageChessBoardCell imageChessBoardCell;
            imageChessBoardCell.x = moment.center.x;
            imageChessBoardCell.y = moment.center.y;
            imageChessBoardCell.area = moment.area;
            imageChessBoardCell.length = moment.length;
            chosenImageChessBoardCellS.insert(imageChessBoardCell);
        }
        else{
            numberOfNotChosenCells++;
            notChosenMoments.push_back(moment);
        }
    }
}

// Callback for extracting Chessboard Cells
void imageCb(const sensor_msgs::ImageConstPtr& msg) {

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

    // Seting up a mask which represents only the needed pixels
    cv::Mat mask = cv_ptr->image.clone();
    mask.setTo(0);



    // Drawin a green circle at the center of each not selected contour moments
    // for(auto &moment : notChosenMoments){
    //    cv::circle(cv_ptr->image, cv::Point(moment.center.x, moment.center.y), 5, CV_RGB(0,255,0)); 
    // }

    int counterCell = 0;

    // Iterating through each selected Cell and putting their values into the mask
    for(auto &imageChessBoardCell : chosenImageChessBoardCellS){

        // Drawing a red circle at the center of the selected contour_moments
        //cv::circle(cv_ptr->image, cv::Point(imageChessBoardCell.x, imageChessBoardCell.y), 5, CV_RGB(255,0,0));

        // Creating the mask box for that cell and setting that pixel value to the actual one
        int distanceToEdge = (imageChessBoardCell.length/8) + offSetCenter;
        int xPixelStart = imageChessBoardCell.x-distanceToEdge;
        int xPixelEnd = imageChessBoardCell.x+distanceToEdge;
        int yPixelStart = imageChessBoardCell.y-distanceToEdge;
        int yPixelEnd = imageChessBoardCell.y+distanceToEdge;


        for(int xPixel = xPixelStart; xPixel <= xPixelEnd; xPixel++){
            for(int yPixel = yPixelStart; yPixel <= yPixelEnd; yPixel++){
                mask.at<cv::Vec3b>(yPixel, xPixel) = cv_ptr->image.at<cv::Vec3b>(yPixel, xPixel);
            }
        }

        // Checking if our vector has elements
        if(imageChessPiecesWithColors.size() != 0){

            // Checking if the cell is occupied
            if(imageChessPiecesWithColors.at(counterCell).imageChessPiece.isOccupied){

                // Get the x and y pos center of the chesspiece and get the color
                int chessPiecePixelX = imageChessPiecesWithColors.at(counterCell).imageChessPiece.x;
                int chessPiecePixelY = imageChessPiecesWithColors.at(counterCell).imageChessPiece.y;

                // Drawing a circle at the center of that chess Piece
                cv::circle(cv_ptr->image, cv::Point(chessPiecePixelX, chessPiecePixelY), 5, CV_RGB(255,0,0));

                cv::Vec3b rgbValue = cv_ptr->image.at<cv::Vec3b>(chessPiecePixelY,chessPiecePixelX);

                int singleValue = rgbValue[0];
                singleValue = (singleValue << 8) + rgbValue[1];
                singleValue = (singleValue << 8) + rgbValue[2];
                if(singleValue < colorChessPieceCuttOfPoint) imageChessPiecesWithColors.at(counterCell).isWhite = false;
                else imageChessPiecesWithColors.at(counterCell).isWhite = true;
            }
            
        }
        

        counterCell++;
    }
    
    // Displaying the image and the mask
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW_Cells, mask);

    cv::waitKey(3);
}



// Callback for getting the depth values and checking if the cell is empty or not
void imageCbDepth(const sensor_msgs::ImageConstPtr& msg) {
    imageChessPiecesWithColors.clear();

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


    ofstream MyFile();

    // Iterating through each selected Chessboard cell
    for(auto &imageChessBoardCell : chosenImageChessBoardCellS){
        
        int totalPixelX = 0;
        int numberOfPixelX = 0;
        int totalPixelY = 0;
        int numberOfPixelY = 0;
        float totalDepth = 0;
        int numberOfDepth = 0;

        // Creating the mask box for that cell and setting that pixel value to the actual one
        int distanceToEdge = (imageChessBoardCell.length/8) + offSetCenter;
        int xPixelStart = imageChessBoardCell.x-distanceToEdge;
        int xPixelEnd = imageChessBoardCell.x+distanceToEdge;
        int yPixelStart = imageChessBoardCell.y-distanceToEdge;
        int yPixelEnd = imageChessBoardCell.y+distanceToEdge;

        for(int xPixel = xPixelStart; xPixel <= xPixelEnd; xPixel++){
            for(int yPixel = yPixelStart; yPixel <= yPixelEnd; yPixel++){

                // Getting the depthValue for that Pixel
                float depthValue = cv_ptr->image.at<float>(yPixel, xPixel);

                // Checking if that pixel depth is small than emptyCell and it minimum
                if(depthValue < (emptyCellDepth - minimumPieceHeight)){
                    totalPixelX = totalPixelX + xPixel;
                    numberOfPixelX++;

                    totalPixelY = totalPixelY + yPixel;
                    numberOfPixelY++;

                    totalDepth = totalDepth + depthValue;
                    numberOfDepth++;
                }
            }
        }

        ImageChessPiece imageChessPiece;

        if(totalPixelX != 0 && totalPixelY != 0 && totalDepth != 0){
            imageChessPiece.x = totalPixelX / numberOfPixelX;
            imageChessPiece.y = totalPixelY / numberOfPixelY;
            imageChessPiece.depth = totalDepth / numberOfDepth;
            imageChessPiece.isOccupied = true;
        }

        ImageChessPieceWithColor imageChessPieceWithColor;
        imageChessPieceWithColor.imageChessPiece = imageChessPiece;

        imageChessPiecesWithColors.push_back(imageChessPieceWithColor);
        

    }
}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "cellDetectionNode");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher chessWrapper_pub = nh.advertise<std_msgs::String>("cellDetection_messages", 10);
    ros::Subscriber corners_sub = nh.subscribe("/ec_goodfeature_filter/corners", 1, &cornersMessageReceived);
    ros::Subscriber moments_sub = nh.subscribe("/contour_moments_edge_detection/moments", 10, &momentsMessageReceived);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber depth_sub = it.subscribe("/external_camera_overhead/ec_depth/image_raw", 5, imageCb);

    image_transport::ImageTransport depth(nh);
    image_transport::Subscriber depth_sub2 = depth.subscribe("/external_camera_overhead/ec_depth/depth/image_raw", 5, imageCbDepth);


    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW_Cells);

    ros::Rate rate(10);


    while(ros::ok()){


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}