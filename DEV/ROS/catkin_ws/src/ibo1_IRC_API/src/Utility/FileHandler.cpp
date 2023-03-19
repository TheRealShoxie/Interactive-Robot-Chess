#include "ibo1_IRC_API/FileHandler.h"

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

    void FileHandler::writeFile(string filepathName, string dataToBeWritten){
        ofstream fileWriter(filepathName);


        if(!fileWriter.is_open()){
            cerr << "File not created!" << endl;
        } else{
            fileWriter << dataToBeWritten;
            fileWriter.close();
        }
    }
    

    string FileHandler::readFile(string filepathName){

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


    vector<User> FileHandler::readUsers(string filepathName){
        string tempString;

        ifstream fileReader(filepathName);
        vector<User> users;


        if(!fileReader.is_open()){
            cerr << "Cannot find file or the file does not exist!" << endl;
        } else{
            while(fileReader.good()){
                getline(fileReader, tempString);
                if(tempString.find("<") == 4){
                    users.push_back(User(tempString));
                }
            }
        }

        return users;
    }