/*
 * FileHandler
 * <p>
 * This file is the implementation of the class definition in FileHandler.h
 * Please refer to FileHandler.h for more information.
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see FileHandler.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

#include "ibo1_irc_api/Utility/FileHandler.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    // Method to write a string of data to a supplied filepath
    // throws invalid_argument if the file does not exist or cannot be found
    void FileHandler::writeFile(string const &filepathName, string const &dataToBeWritten){
        ofstream fileWriter(filepathName);

        // Checking if the fileWriter is not open
        if(!fileWriter.is_open()){

            // Error occured throw an invalid_argument because this comes most likely from incorrectly supplied filepath
            throw std::invalid_argument("'" +filepathName +"'" + " | could not find file or does not exist");
        } 
        // Otherwise write to the file
        else{
            fileWriter << dataToBeWritten;
            fileWriter.close();
        }
    }
    
    // Method to read a file at a specified filePath
    // throws invalid_argument if the file does not exist or cannot be found
    string FileHandler::readFile(string const &filepathName){

        string fileContent, tempString;
        ifstream fileReader(filepathName);

        // Checking if the fileWriter is not open
        if(!fileReader.is_open()){

            // Error occured throw an invalid_argument because this comes most likely from incorrectly supplied filepath
            throw std::invalid_argument("'" +filepathName +"'" + " | could not find file or does not exist");
        } 
        // Otherwise read from the file
        else{

            // While we still have data to read keep reading
            while(fileReader.good()){
                getline(fileReader, tempString);
                fileContent += tempString +"\n";
            }
            fileReader.close();
        }

        // Returns read data as string
        return fileContent;
    }

    // Method to readUsers from a supplied filepath
    // throws invalid_argument if the file does not exist or cannot be found
    vector<User> FileHandler::readUsers(string const &filepathName){
        string readLine;

        string startOfData = "    <username=";
        string dataSplitter = ";";
        string dataStart = "=";
        string dataEnd = ">";
        string adminString = "";
        int end = 0;

        ifstream fileReader(filepathName);
        vector<User> users;
        User tempUser = User();

        // Checking if the fileWriter is not open
        if(!fileReader.is_open()){
            throw std::invalid_argument("'" +filepathName +"'" + " | could not find file or does not exist");
        } 
        // Otherwise read from the file
        else{

            //While we still have data to read
            while(fileReader.good()){

                // Get the current line
                getline(fileReader, readLine);

                // Checking where the User start tag is
                if(readLine.find("<") == 4){
                    
                    // Deleting start of the loadedString
                    readLine.erase(readLine.begin(), readLine.begin() + startOfData.size());

                    // Getting the username and then deleting it plus its separator
                    tempUser.setUsername(DataManipulation::subString(readLine, dataSplitter));
                    DataManipulation::subString(readLine, dataStart);

                    // Getting the password and then deleting it plus its separator
                    tempUser.setPassword(DataManipulation::subString(readLine, dataSplitter));
                    DataManipulation::subString(readLine, dataStart);

                    // Getting the admin
                    adminString = DataManipulation::subString(readLine, dataEnd);
                    if(adminString == "1") tempUser.setAdmin(true);
                    else tempUser.setAdmin(false);

                    users.push_back(tempUser);
                }
            }
        }

        // Returning the read users
        return users;
    }

    // Method to read chessEngines from a specified filepath
    // throws invalid_argument if the file does not exist or cannot be found
    vector<ChessEngineDefinitionStruct> FileHandler::readChessEngines(string const &filePathName){
        string readLine;

        string startOfData = "    <name=";
        string dataSplitter = ";";
        string dataStart = "=";
        string dataEnd = ">";
        string adminString = "";
        int end = 0;

        ifstream fileReader(filePathName);
        vector<ChessEngineDefinitionStruct> chessEngines;
        ChessEngineDefinitionStruct chessEngineDefinitionStruct;

        // Checking if the fileWriter is not open
        if(!fileReader.is_open()){
            throw std::invalid_argument("'" +filePathName +"'" + " | could not find file or does not exist");
        } 
        // Otherwise read from the file
        else{
            while(fileReader.good()){
                getline(fileReader, readLine);

                // CHecking where the ChessEngine starts
                if(readLine.find("<") == 4){
                    // Deleting start of the loadedString
                    readLine.erase(readLine.begin(), readLine.begin() + startOfData.size());

                    // Getting the username and then deleting it plus its separator
                    chessEngineDefinitionStruct.name = DataManipulation::subString(readLine, dataSplitter);
                    DataManipulation::subString(readLine, dataStart);

                    // Getting the password and then deleting it plus its separator
                    chessEngineDefinitionStruct.filePathName = DataManipulation::subString(readLine, dataEnd);

                    chessEngines.push_back(chessEngineDefinitionStruct);

                }

            }
        }

        // Returns the read chess engines
        return chessEngines;
    }