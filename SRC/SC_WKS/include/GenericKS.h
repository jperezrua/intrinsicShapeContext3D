#ifndef HKS_H
#define HKS_H

#include "NeighborMesh.h"
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include "Constante.h"

using namespace Eigen;

#define MT 5 //

class GenericKS{
    public:
        //Constructor/Destructor
        GenericKS();
        GenericKS(char *);
        ~GenericKS();

        //Atributes
        char *name;
        unsigned int linsize;
        Matrix<float, Dynamic, Dynamic> LaplacianMat;  //Laplacian Matrix, old version used *
        VectorXf KS;
        MatrixXf HKSt;
        float MaxKS;
        float MinKS;
        float MeanKS;
        float StdKS;
        //char *lmPointer;                               //old version uses **

        //Member Functions
        void ComputeHKS(NeighborMesh *);
        void ConstructLaplacianMatrixSimple(NeighborMesh *);
        void ConstructLaplacianMatrixSimpleNorm(NeighborMesh *);
        void ConstructLaplacianMatrixVoronoi(NeighborMesh *);
        void HKSEigenDecomposition(void);

        //other
        vector<int> CloserThan(NeighborMesh *, int, float);
        vector<int> CloserTo(NeighborMesh *, int, float, float eps=1);
        vector<int> Between(NeighborMesh *mesh, float C, float D, float eps=0.0);
        float GeodesicDistance(NeighborMesh *,int, int);
        int FindCloserToPoint(NeighborMesh *,int,vector<int>);

        //statistics
        float StdDeviationKS(void);

        //Coloring method
        Vector3d KSToColor(float);
        void SetColorsFromKS(NeighborMesh *);

    private:
        void GetWeightAndArea_ij(NeighborMesh *, int, int, float*, float*);
        float EuclideanDistance(NeighborMesh*, int, int);
        bool isObtuseTriangle(NeighborMesh*, int);
        bool isObtuseAngle(NeighborMesh*, int, int);
        float areaTriangle(NeighborMesh*, int);
        float GetVoronoi_ij(NeighborMesh *, int, int);
};

#endif // HKS_H
