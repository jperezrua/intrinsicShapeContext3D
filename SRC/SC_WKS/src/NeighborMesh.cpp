/***************************************************************************
                                  NeighborMesh.cpp
                             -------------------
 ***************************************************************************/

#include "NeighborMesh.h"
#include <GL/glut.h>
#include <algorithm>
#include "scene.h"
#include "useful.h"
#include <iostream>

using namespace std;
//constructor and destructor

NeighborMesh :: NeighborMesh(){}
NeighborMesh :: ~NeighborMesh(){}

// construction of the various neighborhoods

bool NeighborMesh :: Build_P2P_Neigh()
{
    if ( vertices.empty() || faces.empty() ) return false;

    //first build an array of empty sets

    for( unsigned int i=0; i<vertices.size(); i++)
    {
        set <int> myset;
        P2P_Neigh.push_back(myset);
    }//Hasta aqui todo bien

    // now span all the faces, and adds the points within the corresponding sets

    for( unsigned int i=0; i<faces.size(); i++){

        Vector3i F(faces[i]);

        P2P_Neigh[ F[0] ].insert(F[1]);
        P2P_Neigh[ F[0] ].insert(F[2]);

        //cout<<"FACES: ["<<F[0]<<", "<<F[1]<<", "<<F[2]<<"] "<<"Size="<<P2P_Neigh.size()<<endl;

        P2P_Neigh[ F[1] ].insert(F[0]);
        P2P_Neigh[ F[1] ].insert(F[2]); //Aqui o en la linea anterior

        P2P_Neigh[ F[2] ].insert(F[0]);
        P2P_Neigh[ F[2] ].insert(F[1]);
    }

    return true;
}


void NeighborMesh :: DrawP2P_Neigh( unsigned int i )
{
    if ( P2P_Neigh.empty() || i >= P2P_Neigh.size() )  return;

    glPointSize(5);

    glDisable(GL_LIGHTING);

    //render the considered point in red
    Vector3d P(vertices[i]);

    //Write the number of neighbours in the OGL window
    glColor3f(1,0,0);
    glPushMatrix();
    glTranslatef(P[0], P[1], P[2]);
    char s[255];
    sprintf(s, "%d", (int)P2P_Neigh[i].size());
    string n(s);
    Print3DMessage(0,0,n);
    glPopMatrix();

    glBegin(GL_POINTS);
    glColor3f(1,0,0);
    glVertex3f(P[0], P[1], P[2]);

    //render the neighbors in green
    glColor3f(0,1,0);
    for ( set <int> :: iterator it = P2P_Neigh[i].begin(); it != P2P_Neigh[i].end(); ++it)
    {
        P = vertices[*it];
        glVertex3f(P[0], P[1], P[2]);
    }
    glEnd();
}

bool NeighborMesh :: Build_P2F_Neigh()
{
    if ( vertices.empty() || faces.empty() ) return false;

    //first build an array of empty sets

    for( int unsigned i=0; i<vertices.size(); i++)
    {
        set <int> myset;
        P2F_Neigh.push_back(myset);
    }

    // now span all the faces, and adds the faces within the corresponding sets

    for( unsigned int i=0; i<faces.size(); i++){
        Vector3i F(faces[i]);
        for( int j=0; j<3; j++) P2F_Neigh[F[j]].insert(i);
    }
    return true;
}

void NeighborMesh :: DrawP2F_Neigh( unsigned int i )
{
    if ( P2F_Neigh.empty() || i >= P2F_Neigh.size() )  return;

    glPointSize(5);
    glColor3f(1,0,0);
    glDisable(GL_LIGHTING);

    //render the considered point in red
    Vector3d P(vertices[i]);
    glBegin(GL_POINTS);
    glVertex3f(P[0], P[1], P[2]);
    glEnd();

    //render the neighbor faces in dark green
    glColor3f(0,0.5,0);

    set <int> myset ( P2F_Neigh[i]);

    glEnable(GL_LIGHTING);
    glBegin(GL_TRIANGLES);

    for ( set <int> :: iterator it = myset.begin(); it != myset.end(); it++) Draw_Face_Normal(*it);

    glEnd();

}

bool NeighborMesh :: Build_F2F_Neigh()
{
    if ( vertices.empty() || faces.empty() || P2F_Neigh.empty()) return false;

    for (unsigned int i=0; i<faces.size(); i++){
        //get the face
        Vector3i F(faces[i]);
        set<int> myset;

        //for all the points, get the faces neighbors
        for( int j=0; j<3; j++)
        {
            set_union(

                P2F_Neigh[F[j]].begin(), P2F_Neigh[F[j]].end(),
                myset.begin(), myset.end(),
                insert_iterator< set<int> >(myset,myset.begin())
                );
        }

        //remove the current face
        myset.erase(i);

        //then store in the F2F array
        F2F_Neigh.push_back(myset);
    }

    return true;
}

void NeighborMesh :: DrawF2F_Neigh( unsigned int i )
{
    if(F2F_Neigh.empty() || i>F2F_Neigh.size() ) return;

    //draw the considered face in red

    glBegin(GL_TRIANGLES);
    glColor3f(1,0,0);
    Draw_Face_Normal(i);

    //draw neighbor faces in blue
    glColor3f(0,0,1);
    set <int> myset ( F2F_Neigh[i]);
    for ( set <int> :: iterator it = myset.begin(); it != myset.end(); it++)       Draw_Face_Normal(*it);
    glEnd();
}


set<int> NeighborMesh :: GetP2P_Neigh( int p_index, int n)
{

    set <int> previous; previous.insert(p_index);
    set <int> new_ring = P2P_Neigh[p_index];

    if ( n == 0) return previous;
    if ( n == 1) return new_ring;

    set < int > myset;

    for(int i = 1; i<n; i++)
    {
        // compute the 1 neighbourhood of the previously computed ring

        myset.clear();

        for ( set <int> :: iterator it(new_ring.begin()); it != new_ring.end(); it++)
        {
            for (set<int> :: iterator it2(P2P_Neigh[*it].begin()); it2 != P2P_Neigh[*it].end(); it2++)
            {
            myset.insert(*it2);
            }
        }

        set <int> dum; //seems uneasy to remove elements while updating the set at the same time ==> dum set for performing the boolean difference

        //extract previous from my set
        set_difference( myset.begin(), myset.end(),
                        previous.begin(), previous.end(),
                        insert_iterator< set<int> >(dum,dum.begin())
                        );
        myset = dum; // copy dum result into myset... I whish I could have avoided this

        //previous = myset INTERSECTED with new ring
        previous.clear();
        set_intersection( myset.begin(), myset.end(),
                        new_ring.begin(), new_ring.end(),
                        insert_iterator< set<int> >(previous,previous.begin())
                );

        //new_ring = myset MINUS previous
        new_ring .clear();
        set_difference( myset.begin(), myset.end(),
                        previous.begin(), previous.end(),
                        insert_iterator< set<int> >(new_ring,new_ring.begin())
                        );

    }

    return new_ring;
}

set<int> NeighborMesh :: GetF2F_Neigh( int f_index, int n)
{
    //same principle as for points, except that we are working with faces
    set <int> previous; previous.insert(f_index);
    set <int> new_ring = F2F_Neigh[f_index];

    if ( n == 0) return previous;
    if ( n == 1) return new_ring;

    set < int > myset;

    for(int i = 1; i<n; i++)
    {
        // compute the 1 neighbourhood of the previously computed ring

        myset.clear();

        for ( set <int> :: iterator it(new_ring.begin()); it != new_ring.end(); it++)
        {
            for (set<int> :: iterator it2(F2F_Neigh[*it].begin()); it2 != F2F_Neigh[*it].end(); it2++)
            {
            myset.insert(*it2);
            }
        }
        set <int> dum; //seems uneasy to remove elements while updating the set at the same time ==> dum set for performing the boolean difference

        //extract previous from my set
        set_difference( myset.begin(), myset.end(),
                        previous.begin(), previous.end(),
                        insert_iterator< set<int> >(dum,dum.begin())
                        );
        myset = dum;


        //previous = myset INTER new ring
        previous.clear();
        set_intersection( myset.begin(), myset.end(),
                        new_ring.begin(), new_ring.end(),
                        insert_iterator< set<int> >(previous,previous.begin())
                );

        //new_ring = myset MINUS previous
        new_ring .clear();
        set_difference( myset.begin(), myset.end(),
                        previous.begin(), previous.end(),
                        insert_iterator< set<int> >(new_ring,new_ring.begin())
                        );
    }


    return new_ring;
}

void NeighborMesh ::DrawPoints ( set <int> s)
{
    glBegin(GL_POINTS);
    for( set<int>::iterator it(s.begin()); it != s.end(); it++)
    {
        Vector3d P(vertices[*it]);
        glVertex3f(P[0],P[1],P[2]);
    }
    glEnd();
}

void NeighborMesh :: DrawFaces  ( set <int> s)
{
    for( set<int>::iterator it(s.begin()); it != s.end(); it++)
    {
        Draw_Face_Normal(*it);
    }
}

bool NeighborMesh :: Build_Edges()
{
    if (P2F_Neigh.empty() || faces.empty() || vertices.empty() ) return false;

    for(unsigned int i=0; i<faces.size(); i++)   // for all the faces
    {
        Vector3i F(faces[i]);           // get the face

        for( int j=0; j<3; j++)         // for all the points
        {
            //get index of extremities
            int i1 = F[j];
            int i2 = F[(j+1)%3];

            //sort to guarantee that a same edge won't be represented twice when spanning adjacent face
            if( i1 > i2 ) {int dum(i1); i1=i2; i2=dum;}

            //intersection of both sets of surrounding faces of both extremities
            set <int> myset;

            set_intersection(
                P2F_Neigh[i1].begin(), P2F_Neigh[i1].end(),
                P2F_Neigh[i2].begin(), P2F_Neigh[i2].end(),
                insert_iterator< set<int> >(myset,myset.begin())
                );

            // construct the key as a pair of point index
            pair<int,int> myedge(i1,i2);

            //construct the pair (key, set) and insert into the map
            Edges.insert( pair < pair<int,int>,set<int> > (myedge, myset));
        }
    }
    return true;

}

void NeighborMesh :: DrawEdge( map< pair<int,int>, set<int> > :: iterator it)
{
    pair < pair<int,int>,set<int> > myedge = *it;

    Vector3d A(vertices[myedge.first.first]),B(vertices[myedge.first.second]);

    glDisable(GL_LIGHTING);
    glPointSize(5);

    glBegin(GL_POINTS);
    glColor3f(1,0,0);
    glVertex3f(A[0],A[1],A[2]);
    glColor3f(0,1,0);
    glVertex3f(B[0],B[1],B[2]);
    glEnd();

    glLineWidth(3);
    glColor3f(0,0,1);
    glBegin(GL_LINES);
    glVertex3f(A[0],A[1],A[2]);
    glVertex3f(B[0],B[1],B[2]);
    glEnd();

    glEnable(GL_LIGHTING);
    glColor3f(0.8,0.8,0);
    glBegin(GL_TRIANGLES);
    for(set<int> :: iterator it = myedge.second.begin(); it != myedge.second.end(); it++)
    Draw_Face_Normal(*it);
    glEnd();

}
void NeighborMesh :: DrawEdge(unsigned int i)
{
    if(i > Edges.size() ) return;

    map <pair<int,int>,set<int> > :: iterator it(Edges.begin());
    for(unsigned int j=0;j<i;j++, it++);
    DrawEdge (it);

}

void  NeighborMesh :: DrawBoudaryEdges()
{
    //Illustrative function
    //span edges which have a set a adjacent faces with 0 or 1 element

    map <pair<int,int>,set<int> > :: iterator it;

    glLineWidth(5);
    glColor3f(1,0,0);
    glDisable(GL_LIGHTING);
    for(it = Edges.begin(); it != Edges.end(); it++)
    {
    pair < pair<int,int>,set<int> > myedge = *it;

    if(myedge.second.size()<2)
        {
        Vector3d A(vertices[myedge.first.first]),B(vertices[myedge.first.second]);
        glBegin(GL_LINES);
        glVertex3f(A[0],A[1],A[2]);
        glVertex3f(B[0],B[1],B[2]);
        glEnd();
        }

    if(myedge.second.size()>2)
        {
        Vector3d A(vertices[myedge.first.first]),B(vertices[myedge.first.second]);
        glColor3f(0,1,0);
        glLineWidth(20);
        glBegin(GL_LINES);
        glVertex3f(A[0],A[1],A[2]);
        glVertex3f(B[0],B[1],B[2]);
        glEnd();
        }
    }
}

void NeighborMesh ::BuildDistanceLabels(int A){
    //first build an array to store the minimal distances to reach point i from A
    Labels.clear();
    for(unsigned int i=0; i<vertices.size();i++) Labels.push_back(1e30); // set dummy labels
    Labels[A] = 0; //initialize the distance to 0 for the starting point

    // compute the minimal distance for advancing front
    set<int> advancingfront=P2P_Neigh[A];
    set<int>::iterator it1,it2;
    for(it1 = advancingfront.begin(); it1!=advancingfront.end(); it1++)
        {
            Labels[*it1] = (vertices[A] - vertices[*it1] ).norm();
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
            for(it2=P2P_Neigh[*it1].begin(); it2 != P2P_Neigh[*it1].end(); it2++)
            {
            //compute the distance from it1 to it2. If inferior to D[it2], then update and insert
            dist = (vertices[*it1] - vertices[*it2]).norm();
            if(Labels[*it2] > dist + Labels[*it1]) // this point can be reached with a shorter path
                {
                Labels[*it2] = dist + Labels[*it1];
                advancingfront.insert(*it2);
                ok = true; // new path added, should continue working
                }
            }
        }
    }
}

vector<int> NeighborMesh :: ShortestPath(int A, int B,bool buildlabels)
{
    vector<int> ShortestPath; // set of traversed points

    if(buildlabels)  BuildDistanceLabels(A);

    //now unfold the path from the labels

    ShortestPath.push_back(B);
    int currentpoint = B;

    set<int>::iterator it1;
    // almost always works, except on the infamous dragon : enters an infinite loop
    // actually, using a "while" approach combined with an arithmetic test (if d = d1+d2)
    // is not the most robust way to unfold the shortest path...

    while(currentpoint != A) // while start not reached
    {
        //span the neigh of current point and seek for the previous point
        for(it1 = P2P_Neigh[currentpoint].begin(); it1!=P2P_Neigh[currentpoint].end(); it1++)
        {
            if(
                fabs(
                    (vertices[currentpoint] - vertices[*it1] ).norm() + Labels[*it1]
                        -
                     Labels[currentpoint]
                     ) <1e-12 )
            // avoid if d1 == d2 and replace by if (fabs(d1-d2)<1e-12) to avoid numerical accuracy issues
            {
                currentpoint=*it1;
                ShortestPath.push_back(*it1);
                break;
            }
        }
    }

    return ShortestPath;
}


void NeighborMesh :: SetColorsFromLabels()
{
    // brute force approach assuming that values are uniformly distributed (no outlier)
    if (Labels.empty()) return;

    // get min and max labels
    double Lmin(Labels[0]),Lmax(Labels[0]);
    for(unsigned int i=1; i<Labels.size();i++)
    {
        Lmin = min(Lmin, Labels[i]);
        Lmax = max(Lmax, Labels[i]);
    }

    colors.clear();
    //translate values into [0,1], then convert into color, and store
    for(unsigned int i=0; i<Labels.size();i++)
    {
        colors.push_back( DoubleToColor ( (Labels[i] - Lmin )/(Lmax-Lmin) ) );
        //colors.push_back( Vector3d(1,1,1) );
    }
}

void NeighborMesh :: SetUniformColors(){
    // brute force approach assuming that values are uniformly distributed (no outlier)
    if (Labels.empty()) return;

    colors.clear();
    //translate values into [0,1], then convert into color, and store
    for(unsigned int i=0; i<Labels.size();i++)
        colors.push_back( Vector3d(0,1,1) );
}

void NeighborMesh :: IllustratePointNeighbourhoodComputation(int p_index, int n){
    glPointSize(10);
    for(int i=0; i<n; i++)
    {
        set <int> tmp = GetP2P_Neigh(p_index, i);
        if (i%2 == 0)    glColor3f(1,1-i/double(n),0);
        else glColor3f(0,1-i/double(n),1);
        DrawPoints ( tmp );
    }
}

void NeighborMesh :: IllustrateFaceNeighbourhoodComputation(int f_index, int n)
{

    for(int i=0; i<n; i++)
    {
        set <int> tmp = GetF2F_Neigh(f_index, i);
        if (i%2 == 0)    glColor3f(1,1-i/double(n),0);
        else glColor3f(0,1-i/double(n),1);
        glBegin(GL_TRIANGLES);
        DrawFaces ( tmp );
        glEnd();
    }
}


void  NeighborMesh :: IllustrateEdges( int n)
{
    map< pair<int,int>, set<int> > :: iterator it (Edges.begin());
    int i(0);
    while (i<n)
    {
        DrawEdge(it) ;

        for (unsigned int j=0; j<Edges.size()/n; j++, it++){}
        i++;
    }
}
void  NeighborMesh :: IllustrateP2P_Neigh( int n){
    for(int i=0; i<n; i++)
    {
        DrawP2P_Neigh( i * int(P2P_Neigh.size()/n) );
    }

}

void   NeighborMesh :: IllustrateP2F_Neigh( int n){
    for(int i=0; i<n; i++)
    {
        DrawP2F_Neigh( i * int(P2F_Neigh.size()/n) );
    }
}


void   NeighborMesh :: IllustrateF2F_Neigh( int n){
    for(int i=0; i<n; i++)
    {
        DrawF2F_Neigh( i * int(F2F_Neigh.size()/n) );
    }

}



void   NeighborMesh :: IllustrateShortestPaths (int ngeod, int startpointindex){
    //Recompute everything in case labels have been corrupted
    BuildDistanceLabels(startpointindex);
    SetColorsFromLabels();

    //construct geod approx

     for(int i=1; i< ngeod+1; i++){
        cout<<"Geodesic "<<i<<"...";
        vector<int> mypath = ShortestPath(
            startpointindex,
            i*int(vertices.size()/ngeod - 1)

         );

        if(!mypath.empty())
        {
        Vector3d Col=DoubleToColor((i-1.0)/ngeod);
        glColor3f(Col[0],Col[1], Col[2]);
        glLineWidth(5);
        glDisable(GL_LIGHTING);
        glBegin(GL_LINE_STRIP);
        for(vector<int>::iterator it(mypath.begin()); it != mypath.end(); it++)
            {
           Vector3d P(vertices[*it]);
           glVertex3f(P[0],P[1],P[2]);
            }
        glEnd();
         }
        cout<<"done"<<endl;
    }



}


// returns the index of a vertex of a face for which the associated angle is obtuse
// returns -1 if the triangle is not obtuse

int NeighborMesh :: IsObtuse(unsigned int f_index)
{
    assert ( f_index >=0 && f_index<faces.size() );
    Vector3i F(faces[f_index]);

    for (int i=0; i<3; i++)
    {
        Vector3d V1(vertices[F[(i+1)%3]] - vertices[F[i]]);
        Vector3d V2(vertices[F[(i+2)%3]] - vertices[F[i]]);
        if( V1.dot(V2) < 0 )     return F[i];
    }

    return -1;
}

void NeighborMesh :: GlobalConstruct(void){
    //construct P2P : point to point connectivity
    Build_P2P_Neigh();
    //construct P2F : point to face connectivity
    Build_P2F_Neigh();
    //construct F2F : face to face connectivity
    Build_F2F_Neigh();
    //construct Edges
    Build_Edges();
    ComputeCenter();
}

void NeighborMesh :: ComputeCenter(void){
    bBPmin=vertices[0]; bBPmax=vertices[0];
    for( unsigned int i=0; i<vertices.size(); i++){
        Vector3d P(vertices[i]);
        for( int j=0; j<2; j++){
            bBPmin[j] = min(bBPmin[j],P[j]);
            bBPmax[j] = max(bBPmax[j],P[j]);
        }
        bBCenter += P;
    }

    bBCenter/=vertices.size();
    bBDistance = (bBPmax-bBPmin).norm();
}

//Added by JMP
/*
void NeighborMesh :: ConstructLaplacianMatrix(char *filename){
    this->LaplacianMat = new Matrix<float, Dynamic, Dynamic>;
    this->LaplacianMat->resize(this->vertices.size(), this->vertices.size());
    this->lmPointer = LaplacianMat->data();

    char nfn[255];
    sprintf(nfn, "%s.laplacian", filename);

    std::ifstream ifile(nfn);
    if ( !ifile.is_open() ){
            ifile.close();
            std::ofstream ofile(nfn);
            for (unsigned int i=0;i<this->vertices.size();i++){
                 for (unsigned int j=0; j<this->vertices.size(); j++){
                    if (i==j){
                         this->lmPointer[i + j*this->vertices.size()] = this->P2P_Neigh[i].size();
                    }else{
                        if ( this->P2P_Neigh[i].find(j) != this->P2P_Neigh[i].end() ) this->lmPointer[i + j*this->vertices.size()] = -1;
                        else this->lmPointer[i + j*this->vertices.size()] = 0;

                    }
                    ofile << this->lmPointer[i + j*this->vertices.size()];
                 }
            } ofile<<endl;
            ofile.close();
    }else{
        cout<<"File exists"<<endl;
        for (unsigned int i=0;i<this->vertices.size();i++){
             for (unsigned int j=0; j<this->vertices.size(); j++){
                ifile>>this->lmPointer[i + j*this->vertices.size()];
             }
        }
        ifile.close();
    }

        cout<<"Sample of the matrix: "<<endl<<(this->LaplacianMat->block<15,15>(0,0))<<endl;
}*/

/* OLD VERSION, using C pointers, is faster and less ram consumer, but not Eigen complian :(
void NeighborMesh :: ConstructLaplacianř(void){
    this->lmPointer = new char*[this->vertices.size()];
    for (unsigned int i=0;i<this->vertices.size();i++){
         lmPointer[i]=new char[i+1];         //First, allocate space
         //if (i<7) cout<<"LAPLACIAN["<<i<<"]=\t ";
         for (unsigned int j=0; j<i+1; j++){   //Now fill it
            if (i==j) lmPointer[i][j] = char(this->P2P_Neigh[i].size());
            else{
                if ( this->P2P_Neigh[i].find(j) != this->P2P_Neigh[i].end() ) lmPointer[i][j] = -1;
                else lmPointer[i][j] = 0;
            }
            //if (i<7) cout<<int(lmPointer[i][j])<<"\t ";
         }//if (i<7) cout<<endl;
    }
}

*/
