#ifndef DATACHECKER_H
#define DATACHECKER_H
   
#include <string>
#include <regex>

using namespace std;
   
    // ////////// //
    // Constants. //
    // ////////// //

class DataChecker{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //
        static bool isCorrectMove(string const& moveString){
            regex r("[1-8][a-h][1-8][a-h]");
            smatch match;
            if(regex_search(moveString,match, r)) return true;
            return false;
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

    
};
#endif //DATACHECKER_H