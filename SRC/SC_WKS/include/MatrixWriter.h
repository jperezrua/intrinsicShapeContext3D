#ifndef MATRIXWRITER_H
#define MATRIXWRITER_H

#include <iostream>
#include <stdio.h>

class MatrixWriter{
    public:
        //Destructor/Constructor
        MatrixWriter();
        ~MatrixWriter();
        //Methods
        template<typename T>
        void WriteMatrix(char *filename, T** mat, int cols, int rows){
            FILE *fid;
            if((fid=fopen(filename, "wb"))!=NULL) {
                if(fwrite(mat, sizeof(T), cols*rows, fid) != cols*rows){
                    std::cout<<"Error writing File"<<std::endl;
                }fclose(fid);
            }else std::cout<<"Error opening File "<<filename<<", matrix will not be written."<<std::endl;
        }
};

#endif // MATRIXWRITER_H
