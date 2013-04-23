#ifndef SHAPECONTEXT_H
#define SHAPECONTEXT_H

#include "GenericKS.h"
#include "NeighborMesh.h"
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#define PTS 500

#define HUMAN 0
#define BEAR 1
#define ANT 2

class ShapeContext{
    public:
        ShapeContext();
        virtual ~ShapeContext();

        //atributes
        MatrixXf SC;

        VectorXf SCnorm;
        vector<int> centerpts;
        float MaxR; //Max Radius Shape Context
        float MedR;
        float MinR;

        //Methods
        int FillSC(NeighborMesh *, GenericKS *);
        void SetPoints(NeighborMesh*);

    private:
        vector< complex<double> > DFT(vector< complex<double> >& );
        vector< double > ABS(vector< complex<double> >&);
};

#endif // SHAPECONTEXT_H
