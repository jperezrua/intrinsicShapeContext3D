

#ifndef _NEIGHBORMESH_
#define _NEIGHBORMESH_

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

#include <cassert>
#include <cstring>
#include <vector>
#include <set>
#include <map>
#include <fstream>
using namespace std;

#include "Mesh.h"

class NeighborMesh : public Mesh{
    public:

    //constructor and destructor
    NeighborMesh();
    virtual ~NeighborMesh();

    //attributes
    double bBDistance;
    Vector3d bBCenter;
    Vector3d bBPmin, bBPmax;

    vector < set<int> > P2P_Neigh;
    vector < set<int> > P2F_Neigh;
    vector < set<int> > F2F_Neigh;

    vector<double> Labels;
    map < pair <int,int>,  set<int> > Edges;

    inline map < pair <int,int>,  set<int> > :: iterator Get_Edge(int i1, int i2)
    {
        pair <int, int> mypair;
        if (i1<i2) {mypair.first = i1; mypair.second = i2; return Edges.find(mypair);}
        else {mypair.first = i2; mypair.second = i1; return Edges.find(mypair);}
    }

    //construction of the previous attributes once the file is loaded

    bool Build_P2P_Neigh();
    bool Build_P2F_Neigh();
    bool Build_F2F_Neigh();
    bool Build_Edges();
    void BuildDistanceLabels(int A);
    void GlobalConstruct(void);

    //rendering functions for verification

    void DrawP2P_Neigh( unsigned int i );
    void DrawP2F_Neigh( unsigned int i );
    void DrawF2F_Neigh( unsigned int i );
    void DrawEdge( unsigned int i);
    void DrawEdge( map< pair<int,int>, set<int> > :: iterator it);
    void DrawBoudaryEdges();
    vector<int> ShortestPath(int, int, bool buildlabels=false);
    void SetColorsFromLabels();
    void SetUniformColors();
    void SetColorsFromKean(double n = 5);

    //compute extended neighborhoods

    set<int> GetP2P_Neigh( int, int );
    set<int> GetF2F_Neigh( int, int );

    //drawing functions

    void DrawPoints ( set <int> );
    void DrawFaces  ( set <int> );

    void  IllustratePointNeighbourhoodComputation(int,int);
    void  IllustrateFaceNeighbourhoodComputation(int,int);
    void  IllustrateEdges( int n);
    void  IllustrateP2P_Neigh( int n);

    void  IllustrateP2F_Neigh( int n );
    void  IllustrateF2F_Neigh( int n );
    void  IllustrateShortestPaths (int ngeod, int startpointindex);

    int IsObtuse( unsigned int face_index);
    void ComputeCenter(void);

    /*//added my JMP
    Matrix<float, Dynamic, Dynamic> *LaplacianMat;
    float *lmPointer; //old version uses **
    void ConstructLaplacianMatrix(char *);*/
};

#endif // _NEIGHBORMESH_
