#ifndef DATAMANIPULATION_H
#define DATAMANIPULATION_H

/*
 * DataManipulation
 * <p>
 * Allows the manipulation for data. This is mainly used for getting substring using splitter and a string to extract from
 * 
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see FileHandler.h
 * @see DataCreator.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

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

        // Extracting a substring from a supplied string with a splitter
        // The supplied string will be manipulated and the extracted subString will be removed from it
        static string subString(string &stringToExtract, string splitter){
            int end;
            string returnString;

            // Getting the position of the splitter
            end = stringToExtract.find(splitter);
            // Getting the substring up till the position of the splitter
            returnString = stringToExtract.substr(0, end);

            // Deleting the subString that has been extracted
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