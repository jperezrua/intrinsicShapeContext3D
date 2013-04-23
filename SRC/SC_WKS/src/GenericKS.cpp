#include "GenericKS.h"
#include <iostream>


using namespace std;

GenericKS::GenericKS(){
    name = new char[30];
    sprintf(name, "%s", "generic.wks");
}

GenericKS::GenericKS(char * name){
    this->name = name;;
}

GenericKS::~GenericKS(){
}

void GenericKS::ComputeHKS(NeighborMesh *mesh){
    char nfn[255];
    sprintf(nfn, "%s.wks", this->name);
    std::ifstream ifile(nfn);

    this->linsize=mesh->vertices.size();
    this->HKSt=MatrixXf::Zero(MT,this->linsize);
    this->KS=VectorXf::Zero(this->linsize);

    if ( !ifile.is_open() ){
        cout<<"Computing WKS."<<endl;
        this->ConstructLaplacianMatrixSimple(mesh);
        this->HKSEigenDecomposition();
        ifile.close();
        std::ofstream ofile(nfn);
        cout<<"Creating WKS file for this mesh."<<endl;
        for (uint i=0;i<MT;i++){
            for (uint j=0;j<this->linsize;j++){
                char towrite[100];
                sprintf(towrite, "%f,",HKSt(i,j));
                ofile<<towrite<<endl;
            }
        }
        ofile.close();
    }else{
        cout<<"WKS file already exists"<<endl;
        char toread[100]="";
        for (unsigned int i=0;i<MT;i++){
            for (unsigned int l=0;l<this->linsize;l++){
                char item='a'; int j=0;
                ifile>>item;
                while(item!=','){
                    toread[j]=item;
                    ifile>>item;
                    j++;
                }toread[j]='\0';
                float t;
                sscanf(toread, "%f", &t);
                HKSt(i,l) = t;
            }
        }ifile.close();
        cout<<"Verify this values: "<<HKSt(0,0)<<", "<<HKSt(0,1)<<endl;
    }

    //Times to analyze
    uint I=0;
    uint F=4;
    //Reference Point (Always the first one)
    uint R=0;
    KS(R) = abs(HKSt(F,R) - HKSt(I,R));
    for (unsigned int l=0;l<this->linsize;l++){
        KS(l) = abs( KS(R)-abs(HKSt(F,l) - HKSt(I,l)) ); //Reference Point - Others Points
    }

    this->MaxKS = KS.maxCoeff();
    this->MinKS = KS.minCoeff();
    this->MeanKS= KS.mean();
    this->StdKS = StdDeviationKS();

    double slope = 1/(this->MaxKS-this->MinKS);
    for(uint i=0; i<KS.size();i++)
        KS(i)= slope*( this->KS(i) - this->MinKS );

    /*for (uint i=0;i<this->linsize;i++){
        if (KS(i) > MeanKS+7500*StdKS ){
            KS(i)=1;
        }
        if (KS(i) < MeanKS-7500*StdKS ){
            KS(i)=0;
        }
    }*/

    this->HKSt.resize(1,1);

    this->MaxKS = KS.maxCoeff(); cout<<"Max KS is: " <<this->MaxKS<<endl;
    this->MinKS = KS.minCoeff(); cout<<"Min KS is: " <<this->MinKS<<endl;
    this->MeanKS= KS.mean();    cout<<"Mean KS is: " <<this->MeanKS<<endl;
    this->StdKS = StdDeviationKS(); cout<<"StdDev KS is: " <<this->StdKS<<endl;
}

float GenericKS::StdDeviationKS(void){
    float u = KS.mean();
    float d=0;
    for (uint i=0;i<KS.size(); i++){
        d += (KS(i)-u)*(KS(i)-u);
    }d = sqrt(d/KS.size());
    return d;
}

void GenericKS::HKSEigenDecomposition(void){
    //Computing EigenValues and EigenVectors
    //compute evd for a symmetric matrix
    cout<<"Eigendecomposition starting now..."<<endl;

    VectorXf EVals;
    SelfAdjointEigenSolver< MatrixXf > gksSolver( this->LaplacianMat ); //this->LaplacianMat.cast<float>() for old

    cout<<"HKS starting..."<<endl;
    EVals = gksSolver.eigenvalues();
    //EVals = gksSolver.eigenvalues()/(gksSolver.eigenvalues().sum()); //nah, Normalizing eigenvalues for a better scale invariancy
    cout<<"EVals Created: "<<EVals.size()<<endl;
    //sort(EVals.data(), EVals.data()+EVals.size());
    //cout<<"EVals Sorted: "<<EVals(1)<<", "<<EVals(2)<<EVals(EVals.size()-1)<<endl;

    MatrixXf V;
    cout<<"Allocating V: ";
    cout<<"Done."<<endl;
    V = gksSolver.eigenvectors();
    //V = V/V.sum(); //thid did not worked well
    cout<<"V now contains  eigenvectors"<<endl;

    //WKS for a File that does not exist
    cout<<"HKS computed for different times: "<<endl;

    float t=100;
    float inc=1000/MT;

    MatrixXf E = EVals.asDiagonal();
    cout<<"Matrix E initialized "<<endl;
    HKSt.resize( MT, EVals.size() );
    for (unsigned int k=0; k<MT; k++){
        cout<<"t = "<<t<<endl;
        for (uint i=0; i<this->linsize;i++) E(i,i) = exp(-t*EVals(i));
        this->KS = ( V * E * V.transpose() ).diagonal();
        for (uint c=0; c<EVals.size(); c++) this->HKSt(k,c)=this->KS(c);
        t=t+inc;
    }
}

void GenericKS::ConstructLaplacianMatrixSimpleNorm(NeighborMesh *mesh){
    this->linsize = mesh->vertices.size();
    cout<<"Constructing simple norm. Laplacian for: "<<this->linsize<<". "<<endl;
    this->LaplacianMat.resize(mesh->vertices.size(), mesh->vertices.size());

    VectorXf D;
    D.resize(mesh->vertices.size());
    //LaplacianMat = Adjacency Matrix; D = diagonal
    for (unsigned int i=0;i<mesh->vertices.size();i++){
         for (unsigned int j=0; j<mesh->vertices.size(); j++){
            if (i==j){
                 D(i) = mesh->P2P_Neigh[i].size();
                 this->LaplacianMat(i,j) = 0;
            }else{
                if ( mesh->P2P_Neigh[i].find(j) != mesh->P2P_Neigh[i].end() ) this->LaplacianMat(i,j)=-1;
                else this->LaplacianMat(i,j)=0;
            }
         }
    }
    //D=pow(D,-0.5), LaplacianMat = D*A*D
    for (unsigned int i=0;i<mesh->vertices.size();i++) D(i)=pow(D(i),-0.5);
    this->LaplacianMat = this->LaplacianMat*D.asDiagonal();
    this->LaplacianMat = D.asDiagonal()*this->LaplacianMat;
    //LaplacianMat = I - LaplacianMat;
    for (unsigned int i=0;i<mesh->vertices.size();i++) LaplacianMat(i,i)=1-LaplacianMat(i,i);
    //cout<<"First 50x10 matrix "<<endl<<LaplacianMat.block<50,10>(0,0)<<endl;
}

void GenericKS::ConstructLaplacianMatrixVoronoi(NeighborMesh *mesh){
    this->linsize = mesh->vertices.size();
    cout<<"Constructing Laplacian for "<<this->linsize<<". "<<endl;
    this->LaplacianMat.resize(mesh->vertices.size(), mesh->vertices.size());

    for (unsigned int i=0;i<mesh->vertices.size();i++){
         for (unsigned int j=0; j<mesh->vertices.size(); j++){
            if (i==j){
                 float weight=0.0;
                 float area=0.0;
                 float val=0.0;
                 for (unsigned int k=0; k<mesh->vertices.size(); k++){
                    if ( mesh->P2P_Neigh[i].find(k) != mesh->P2P_Neigh[i].end() ){
                        GetWeightAndArea_ij(mesh,i,k,&weight,&area);
                        val+=weight;
                    }
                 }val=val/area; if (isinf(val) || isnan(val) || val==0) val=mesh->P2P_Neigh[i].size();
                 this->LaplacianMat(i,j) = val;
            }else{
                if ( mesh->P2P_Neigh[i].find(j) != mesh->P2P_Neigh[i].end() ){
                 float weight=0.0;
                 float area=0.0;
                 GetWeightAndArea_ij(mesh,i,j,&weight,&area);
                 this->LaplacianMat(i,j)=-weight/area;
                 if (isnan(this->LaplacianMat(i,j))) this->LaplacianMat(i,j)=-1;
                }
                else this->LaplacianMat(i,j)=0;
            }
            if (isnan(this->LaplacianMat(i,j)));
         }
    }//cout<<"First 10x10 matrix "<<endl<<LaplacianMat.block<50,10>(0,0)<<endl;
}

void GenericKS::ConstructLaplacianMatrixSimple(NeighborMesh *mesh){
    this->linsize = mesh->vertices.size();
    cout<<"Constructing Laplacian for "<<this->linsize<<". "<<endl;
    //this->LaplacianMat = new Matrix<char, Dynamic, Dynamic>; //old
    this->LaplacianMat.resize(mesh->vertices.size(), mesh->vertices.size());
    //this->lmPointer = this->LaplacianMat.data();

    for (unsigned int i=0;i<mesh->vertices.size();i++){
         for (unsigned int j=0; j<mesh->vertices.size(); j++){
            if (i==j){
                 //this->lmPointer[i + j*mesh->vertices.size()] = mesh->P2P_Neigh[i].size();
                 this->LaplacianMat(i,j) = mesh->P2P_Neigh[i].size();
                 //cout<<"%"<<100*(float(i)/mesh->vertices.size())<<"\t";
            }else{
                if ( mesh->P2P_Neigh[i].find(j) != mesh->P2P_Neigh[i].end() ) this->LaplacianMat(i,j)=-1;
                //this->lmPointer[i + j*mesh->vertices.size()] = -1;
                else this->LaplacianMat(i,j)=0;
                //this->lmPointer[i + j*mesh->vertices.size()] = 0;
            }
         }
    }cout<<endl;
}

Vector3d GenericKS::KSToColor( float d ){
	if(d<0) return Vector3d(0,0,0);
	if(0<=d && d<0.25)		{ return Vector3d(0 , d*4.0 ,1);}
	if(0.25<=d && d<0.5)	{ return Vector3d(0 , 1.0 , 2.0-4.0*d);}
	if(0.5<=d && d<0.75)	{ return Vector3d(4.0*d - 2.0 , 1.0   ,0);	}
	if(0.75<=d && d<=1.0)	{ return Vector3d(1, 4.0-4.0*d  ,0);}
	return Vector3d(1,0,0);
}

void GenericKS :: SetColorsFromKS(NeighborMesh *mesh){
    mesh->colors.clear();
    mesh->colors.push_back( Vector3d(1,1,1) );
    for(unsigned int i=1; i<KS.size();i++)
        mesh->colors.push_back( this->KSToColor(KS(i)) );
}

vector<int> GenericKS ::CloserTo(NeighborMesh *mesh, int A, float D, float eps){
    vector< int > out;
    for(unsigned int i=0; i<mesh->vertices.size();i++){
        if ( mesh->Labels[i]-eps<=D && mesh->Labels[i]+eps>=D ){
            //cout<<"LABEL"<<i<<",: "<<mesh->Labels[i]<<endl;
            out.push_back(i);
        }
    }
    return out;
}

vector<int> GenericKS ::Between(NeighborMesh *mesh, float C, float D, float eps){
    vector< int > out;
    for(unsigned int i=0; i<mesh->vertices.size();i++){
        if ( mesh->Labels[i]<=D+eps && mesh->Labels[i]>=C-eps ){
            out.push_back(i);
        }
    }
    return out;
}

int GenericKS::FindCloserToPoint(NeighborMesh *mesh,int A,vector<int> group){
    vector<float> Labels;
    Labels.clear();
    for(unsigned int i=0; i<mesh->vertices.size();i++) Labels.push_back(1e30); // set dummy labels
    Labels[A] = 0; //initialize the distance to 0 for the starting point

    // compute the minimal distance for advancing front
    set<int> advancingfront=mesh->P2P_Neigh[A];
    set<int>::iterator it1,it2;
    for(it1 = advancingfront.begin(); it1!=advancingfront.end(); it1++)
        {
            Labels[*it1] = (mesh->vertices[A] - mesh->vertices[*it1] ).norm();
        }

    bool ok=true; double dist;
    set<int> dumset;
    while(ok)
    {   ok = false; // suppose algo should stop
        //compute the new points to be evaluated
        dumset = advancingfront;
        advancingfront.clear();

        //compute and update the distance from existing points to advancing front
        //add the P2P neigh, if required, of the advancing front to dumset
        for(it1 = dumset.begin(); it1!=dumset.end(); it1++)
        {
            for(it2=mesh->P2P_Neigh[*it1].begin(); it2 != mesh->P2P_Neigh[*it1].end(); it2++)
            {
            //compute the distance from it1 to it2. If inferior to D[it2], then update and insert
            dist = (mesh->vertices[*it1] - mesh->vertices[*it2]).norm();
            if(Labels[*it2] > dist + Labels[*it1]) // this point can be reached with a shorter path
                {
                Labels[*it2] = dist + Labels[*it1];
                advancingfront.insert(*it2);
                ok = true; // new path added, should continue working
                }
            }
        }
    }
    int min=group[0];
    for (uint i=1;i<group.size();i++){
        if (Labels[group[i]]<Labels[min]){
            min=Labels[group[i]];
        }
    }
    return min;
}

vector<int> GenericKS ::CloserThan(NeighborMesh *mesh, int A, float D){
    vector< int > out;
    for(unsigned int i=0; i<mesh->vertices.size();i++){
        if ( float(mesh->Labels[i]) <=D ){
            out.push_back(i);
        }
    }
    return out;
}

float GenericKS::GeodesicDistance(NeighborMesh *mesh,int A, int B){
    //first build an array to store the minimal distances to reach point i from A
    vector<float> Labels;
    for(unsigned int i=0; i<mesh->vertices.size();i++) Labels.push_back(1e30); // set dummy labels
    Labels[A] = 0; //initialize the distance to 0 for the starting point

    // compute the minimal distance for advancing front
    set<int> advancingfront=mesh->P2P_Neigh[A];
    set<int>::iterator it1,it2;
    for(it1 = advancingfront.begin(); it1!=advancingfront.end(); it1++)
        {
            Labels[*it1] = (mesh->vertices[A] - mesh->vertices[*it1] ).norm();
        }

    bool ok=true; double dist;
    set<int> dumset;
    while(ok)
    {   ok = false; // suppose algo should stop
        //compute the new points to be evaluated
        dumset = advancingfront;
        advancingfront.clear();

        //compute and update the distance from existing points to advancing front
        //add the P2P neigh, if required, of the advancing front to dumset
        for(it1 = dumset.begin(); it1!=dumset.end(); it1++)
        {
            for(it2=mesh->P2P_Neigh[*it1].begin(); it2 != mesh->P2P_Neigh[*it1].end(); it2++)
            {
            //compute the distance from it1 to it2. If inferior to D[it2], then update and insert
            dist = (mesh->vertices[*it1] - mesh->vertices[*it2]).norm();
            if(Labels[*it2] > dist + Labels[*it1]) // this point can be reached with a shorter path
                {
                Labels[*it2] = dist + Labels[*it1];
                advancingfront.insert(*it2);
                ok = true; // new path added, should continue working
                }
            }
        }
    }
    return Labels[B];
}

//As used on Meyer et al 2001
void GenericKS::GetWeightAndArea_ij(NeighborMesh *mesh, int i/*ref point*/, int j, float* mij, float* si){
    set<int> set_i = mesh->P2P_Neigh[i];
    set<int> set_j = mesh->P2P_Neigh[j];
    set<int> intersect;

    //intersection of sets i and j
    set_intersection(set_i.begin(), set_i.end(), set_j.begin(), set_j.end(), inserter(intersect, intersect.end()));
    int jm = *(intersect.begin());
    int jM = *(intersect.end());

    //cout<<"jm: "<<jm<<", "<<"jM: "<<jM<<endl;

    float AI=EuclideanDistance(mesh, jm, i), IJ=EuclideanDistance(mesh, jm, i);
    float JB=EuclideanDistance(mesh, j, jM), IB=EuclideanDistance(mesh, i, jM);
    float AJ=EuclideanDistance(mesh, jm, j);

    //cout<<"Distances: "<<AI<<", "<<AJ<<", "<<JB<<", "<<IB<<", "<<IJ<<endl;
    //cout<<"Computing: alpha = acos("<<(AI*AI+AJ*AJ-IJ*IJ)/(2*AI*AJ)<<")"<<endl;
    //cout<<"Computing: betha = acos("<<(IB*IB+JB*JB-IJ*IJ)/(2*IB*JB)<<")"<<endl;

    float alpha = acos((AI*AI+AJ*AJ-IJ*IJ)/(2*AI*AJ));
    float betha = acos((IB*IB+JB*JB-IJ*IJ)/(2*IB*JB));
    float cotalpha = 1/tan(alpha);
    float cotbetha = 1/tan(betha);

    //cout<<"angles: "<<alpha<<", "<<betha<<", "<<cotalpha<<", "<<cotbetha<<endl;

    if (!isnan(cotalpha) && !isnan(cotbetha)){
        *mij = (cotalpha+cotbetha)/2; //weight ij: output variable mij
    }else if (isnan(cotalpha) && !isnan(cotbetha)){
        *mij=cotbetha;
        //cout<<"NAN1"<<endl;
    }else if (isnan(cotbetha) && !isnan(cotalpha)){
        *mij=cotalpha;
        //cout<<"NAN2"<<endl;
    }else if (isnan(cotbetha) &&  isnan(cotalpha)){
        *mij=0;
        //cout<<"NAN3"<<endl;
    }

    *si = 0; //infinitesimal area initialization

    //Now, for the voronoi area, we have to check wheter each triangle is obtuse or not
    set<int> facesfori = mesh->P2F_Neigh[i];
    set<int>::const_iterator it;
    for (it=facesfori.begin(); it!=facesfori.end(); ++it){
        if (isObtuseTriangle(mesh, *it)){
            if (isObtuseAngle(mesh, i, *it)){
                *si+=areaTriangle(mesh, *it)/2;
            }else{
                *si+=areaTriangle(mesh, *it)/4;
            }
        }else{
            *si+=GetVoronoi_ij(mesh, i, *it);
        }
    }

    if (isnan(*si)){ *si=1;  }
}

float GenericKS::GetVoronoi_ij(NeighborMesh* mesh, int i/*number of the vertex*/,  int j/*face*/){
    Vector3i face = mesh->faces[j];
    float OP=0, PR=0, PQ=0, q=0, r=0;

    if (face[0]==i){
        OP = EuclideanDistance(mesh, face[1], face[2]);
        PR = EuclideanDistance(mesh, face[0], face[1]);
        PQ = EuclideanDistance(mesh, face[0], face[2]);
    }else if(face[1]==i){
        OP = EuclideanDistance(mesh, face[0], face[2]);
        PR = EuclideanDistance(mesh, face[1], face[2]);
        PQ = EuclideanDistance(mesh, face[1], face[0]);
    }else if(face[2]==i){
        OP = EuclideanDistance(mesh, face[1], face[0]);
        PR = EuclideanDistance(mesh, face[2], face[1]);
        PQ = EuclideanDistance(mesh, face[2], face[0]);
    }
    q = 1/tan(acos((PQ*PQ+OP*OP-PR*PR)/(2*PQ*OP)));
    r = 1/tan(acos((PR*PR+OP*OP-PQ*PQ)/(2*PR*OP)));

    //cout<<"atan(q)="<<q<<", atan(r)="<<r<<endl;
    if (isnan(q)) q=0;
    if (isnan(r)) r=0;

    return (PR*PR*q+PQ*PQ*r)/8;
}

bool GenericKS::isObtuseAngle(NeighborMesh* mesh, int i/*number of the vertex*/,  int j/*face*/){
    Vector3i face = mesh->faces[j];
    float OP=0, AD1=0, AD2=0;

    if (face[0]==i){
        OP = EuclideanDistance(mesh, face[1], face[2]);
        AD1 = EuclideanDistance(mesh, face[0], face[1]);
        AD2 = EuclideanDistance(mesh, face[0], face[2]);
    }else if(face[1]==i){
        OP = EuclideanDistance(mesh, face[0], face[2]);
        AD1 = EuclideanDistance(mesh, face[1], face[2]);
        AD2 = EuclideanDistance(mesh, face[1], face[0]);
    }else if(face[2]==i){
        OP = EuclideanDistance(mesh, face[1], face[0]);
        AD1 = EuclideanDistance(mesh, face[2], face[1]);
        AD2 = EuclideanDistance(mesh, face[2], face[0]);
    }
    float alpha = acos((AD1*AD1+AD2*AD2-OP*OP)/(2*AD1*AD2));

    return (alpha>PI/2)?true:false;
}

float GenericKS::areaTriangle(NeighborMesh* mesh, int i/*number of the face*/){
    Vector3i face = mesh->faces[i];

    float A = EuclideanDistance(mesh, face[1], face[2]);
    float B = EuclideanDistance(mesh, face[0], face[2]);
    float C = EuclideanDistance(mesh, face[0], face[1]);

    float S = (A+B+C)/2;

    return ( sqrt(S*(S-A)*(S-B)*(S-C)) );
}

bool GenericKS::isObtuseTriangle(NeighborMesh* mesh, int i/*number of the face*/){
    Vector3i face = mesh->faces[i];

    float A = EuclideanDistance(mesh, face[1], face[2]);
    float B = EuclideanDistance(mesh, face[0], face[2]);
    float C = EuclideanDistance(mesh, face[0], face[1]);

    float a = abs(acos((B*B+C*C-A*A)/(2*B*C)));
    float b = abs(acos((A*A+C*C-B*B)/(2*A*B)));
    float c = abs(acos((B*B+A*A-C*C)/(2*A*B)));

    return (a>PI/2||b>PI/2||c>PI/2)?true:false;
}

//euclidean distance given 2 indexes of vertices in a given mesh
float GenericKS::EuclideanDistance(NeighborMesh* mesh, int i, int j){
    Vector3d vi = mesh->vertices[i];
    Vector3d vj = mesh->vertices[j];

    return ( sqrt( pow(vi(0)-vj(0),2) + pow(vi(1)-vj(1),2) + pow(vi(2)-vj(2),2) ) );
}

