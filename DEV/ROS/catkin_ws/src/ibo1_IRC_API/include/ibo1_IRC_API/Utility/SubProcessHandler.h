#ifndef SUBPROCESSHANDLER_H
#define SUBPROCESSHANDLER_H

#include <ext/stdio_filebuf.h> // NB: Specific to libstdc++
#include <sys/wait.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <exception>
#include <istream>
#include <vector>

using namespace std;

/*
    - This class was written using https://gist.github.com/konstantint/d49ab683b978b3d74172,
      this enables the starting of a subprocess and handling in and output.
*/


// The following classes were taken from the above mentioned link. These are used to enable the functionality of the above class.
// Wrapping pipe in a class makes sure they are closed when we leave scope
class cpipe {
private:
    int fd[2];
public:
    const inline int read_fd() const { return fd[0]; }
    const inline int write_fd() const { return fd[1]; }
    cpipe() { if (pipe(fd)) throw runtime_error("Failed to create pipe"); }
    void close() { ::close(fd[0]); ::close(fd[1]); }
    ~cpipe() { close(); }
};

class SubProcess {
private:
    cpipe write_pipe;
    cpipe read_pipe;
public:
    int child_pid = -1;
    unique_ptr<__gnu_cxx::stdio_filebuf<char> > write_buf = NULL; 
    unique_ptr<__gnu_cxx::stdio_filebuf<char> > read_buf = NULL;
    ostream stdin;
    istream stdout;

    SubProcess(): stdin(NULL), stdout(NULL){}

    // Contribution from maxim line 58-62
    SubProcess(vector<string> const &argvSource, bool with_path = false, const char* const envp[] = 0): stdin(NULL), stdout(NULL){
        child_pid = fork();
        if (child_pid == -1) throw runtime_error("Failed to start child process"); 
        if (child_pid == 0) {   // In child process
            dup2(write_pipe.read_fd(), STDIN_FILENO);
            dup2(read_pipe.write_fd(), STDOUT_FILENO);
            write_pipe.close(); read_pipe.close();
            int result;

            char ** argv = new char*[argvSource.size() + 1];
            for (size_t i = 0; i < argvSource.size(); ++i) {
                argv[i] = strdupa(argvSource[i].c_str());
            }
            argv[argvSource.size()] = NULL;

            if (with_path) {
                if (envp != 0) result = execvpe(argv[0], const_cast<char* const*>(argv), const_cast<char* const*>(envp));
                else result = execvp(argv[0], const_cast<char* const*>(argv));
            }
            else {
                if (envp != 0) result = execve(argv[0], const_cast<char* const*>(argv), const_cast<char* const*>(envp));
                else result = execv(argv[0], const_cast<char* const*>(argv));
            }
            if (result == -1) {
               // Note: no point writing to stdout here, it has been redirected
               throw runtime_error("Failed to launch program");
               cerr << "Error: Failed to launch program" << endl;
               exit(1);
            }
        }
        else {
            close(write_pipe.read_fd());
            close(read_pipe.write_fd());
            write_buf = unique_ptr<__gnu_cxx::stdio_filebuf<char> >(new __gnu_cxx::stdio_filebuf<char>(write_pipe.write_fd(), ios::out));
            read_buf = unique_ptr<__gnu_cxx::stdio_filebuf<char> >(new __gnu_cxx::stdio_filebuf<char>(read_pipe.read_fd(), ios::in));
            stdin.rdbuf(write_buf.get());
            stdout.rdbuf(read_buf.get());
        }
    }
    
    void send_eof() { write_buf->close(); }
    
    int wait() {
        int status;
        waitpid(child_pid, &status, 0);
        return status;
    }
};



   
   
   
    // ////////// //
    // Constants. //
    // ////////// //

class SubProcessHandler{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        SubProcessHandler();
        SubProcessHandler(string const &processFilePathName);

        // //////// //
        // Methods. //
        // //////// //

        void getLine(string &returnedLine);
        void write(string const &message);
        void closeSubProcess();

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

        SubProcess subProcess;
};

#endif //SUBPROCESSHANDLER_H