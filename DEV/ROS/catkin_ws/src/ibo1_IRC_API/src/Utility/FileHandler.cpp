#include "ibo1_IRC_API/Utility/FileHandler.h"

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

    void FileHandler::writeFile(string const &filepathName, string const &dataToBeWritten){
        ofstream fileWriter(filepathName);


        if(!fileWriter.is_open()){
            cerr << "File not created!" << endl;
        } else{
            fileWriter << dataToBeWritten;
            fileWriter.close();
        }
    }
    

    string FileHandler::readFile(string const &filepathName){

        string fileContent, tempString;
        ifstream fileReader(filepathName);

        

        if(!fileReader.is_open()){
            cerr << "Cannot find file or does not exist!" << endl;
        } else{
            while(fileReader.good()){
                getline(fileReader, tempString);
                fileContent += tempString +"\n";
            }
            fileReader.close();
        }

        return fileContent;
    }


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


        if(!fileReader.is_open()){
            cerr << "Cannot find file or the file does not exist!" << endl;
        } else{
            while(fileReader.good()){
                getline(fileReader, readLine);
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

        return users;
    }

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

        if(!fileReader.is_open()){
            cerr << "Cannot find file or the file does not exist!" << endl;
        } else{
            while(fileReader.good()){
                getline(fileReader, readLine);
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

        return chessEngines;
    }