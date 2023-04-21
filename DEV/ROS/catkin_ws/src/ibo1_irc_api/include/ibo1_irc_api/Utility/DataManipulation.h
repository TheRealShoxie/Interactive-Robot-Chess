#ifndef DATAMANIPULATION_H
#define DATAMANIPULATION_H

#include <string>

using namespace std;
   
    // ////////// //
    // Constants. //
    // ////////// //

class DataManipulation{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //
        static string subString(string &stringToExtract, string splitter){
            int end;
            string returnString;

            end = stringToExtract.find(splitter);
            returnString = stringToExtract.substr(0, end);
            stringToExtract.erase(stringToExtract.begin(), stringToExtract.begin() + end + splitter.size());

            return returnString;
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
#endif //DATAMANIPULATION_H