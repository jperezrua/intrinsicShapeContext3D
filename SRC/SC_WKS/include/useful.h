/***************************************************************************
                                  useful.h
                             -------------------
 ***************************************************************************/




#ifndef _USEFUL_
#define _USEFUL_

#include <cstring>
#include <vector>
#include <fstream>
#include <iostream>
using namespace std;

#include "Mesh.h"
#include <ctime>
#include "Constante.h"

#include <Eigen/Core>
using namespace Eigen;

    unsigned int mySplit(const std::string &, std::vector<std::string> &, char); //TEST FUNCTION
    bool ReadPly( Mesh&, const string& );
    bool ReadOff( Mesh&, const string& );
    // convertion RGB <--> value between [0,1]
	Vector3d DoubleToColor( double d );
	double ColorToDouble(Vector3d RGB);
	Vector3d DoubleToColorDiscrete( const double d, double n =20);

    //OpenGL text display
	void PrintMessage(int,int, const string);

    //mesh perturbation if you want to play a little bit
    void AddGaussianNoise(vector<Vector3d> & vertices, vector<Vector3d> normals, double sigma,double mu=0);

    //read and write 3D files
    bool ReadIv( Mesh& mesh, const string& file_name );
    bool WriteIv( const Mesh& mesh, const string& file_name, const bool& vrml1 );

    //miscellanous
    void ScreenShot(string File); // from opengl buffer to infamous .tga export
#endif



