#include <iostream>
#include <fstream>
#include "scene.h"
#include "Constante.h"
#include "useful.h"
#include "Stopwatch.h"
#include <time.h>
#include <string>
#include "GenericKS.h"
#include "NeighborMesh.h"
#include "ShapeContext.h"


using namespace std;

int parse_cmd(int argc, char** argv,
	      char* file3D){

  int i;
  for(i = 1; i < argc; i++){
    if((std::strcmp(argv[i],"-?") == 0) ||
       (std::strcmp(argv[i],"--help") == 0)){
      std::cout << "SC_WKS: Written by Juan M. Pérez, and Jilliam M. Díaz." << std::endl
	   << "Performs shape context using wave kernel signature on 3D models." << std::endl << std::endl
	   << "#" << std::endl
	   << "# usage: ./SC_WKS [options]" << std::endl
	   << "#" << std::endl << std::endl
	   << "Arguments:" << std::endl
	   << "-m <string> -> 3D Model "
	   << std::endl;
      return -1;
    }
  }

  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-m") == 0){
      if(argc > i+1)std::strcpy(file3D,argv[i+1]);
      else strcpy(file3D,"../../Resampled/ant2.ply");
      break;
    }
  }if(i >= argc) std::strcpy(file3D,"../../Resampled/ant1.ply");

  return 0;
}

int main(int argc,char ** argv){

    Stopwatch timer;   //measure time
    cout<<"Starting..."<<endl;
    //openGL initialization
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA |GLUT_DEPTH);
	glutInitWindowSize(800,600);
	glutInitWindowPosition(10,10);
	glutCreateWindow("VIEWER");
	glutReshapeFunc(reshape); // function call when reshape the window
	glutMotionFunc(Motion);   // function called when mouse move
	glutMouseFunc(Mouse);     // function called when mouse click
	glutKeyboardFunc(Keyboard); // if key pressed
	glutSpecialFunc(Special);   // if special key pressed
	glutDisplayFunc(display);   // display function

    extern scene My_Scene;    // class to handle lights, position, etc
    extern NeighborMesh globalmesh;   // yes another infamous global variable
    extern int id_globalmesh; // identifier for display list. See function display() in scene.cpp

	Init();

    char file3D[100];
    parse_cmd(argc, argv, file3D);
    timer.Reset(); timer.Start();
    if (!globalmesh.ReadFile(file3D)) exit(0);
    timer.Stop();
    cout<<"Loading time :"<< timer.GetTotal()/1000.0<<" ms"<<endl;

    //construct connectivity P2P, P2F, F2F, and edges
    globalmesh.GlobalConstruct();
    timer.Reset(); timer.Start();
    GenericKS hkscomputer(file3D);
    //compute the HKS for the mesh globalmesh
    hkscomputer.ComputeHKS(&globalmesh);
    timer.Stop();
    cout<<"WKS Constructing Time :"<< timer.GetTotal()/1000.0/60<<" secs"<<endl;

    //roughly adjust view frustrum to object and camera position
    My_Scene.Camera_Target = globalmesh.bBCenter;

    //set arbitraty position to camera and adjusts max and min view planes
    My_Scene.Camera_Position = My_Scene.Camera_Target + Vector3d(globalmesh.bBDistance,0,globalmesh.bBDistance);
    My_Scene.znear = globalmesh.bBDistance*0.1;
    My_Scene.zfar = globalmesh.bBDistance*5;

    //adjust displacements consequently
    My_Scene.Object_Move[0]=My_Scene.Object_Move[1]=My_Scene.Object_Move[2] = globalmesh.bBDistance/20;

    //creates lights accordingly to the bounding box of the object
    My_Scene.Create_Lighting(globalmesh.bBPmax,globalmesh.bBPmin, My_Scene.Camera_Position, My_Scene.Camera_Target);

    //compute normals
    globalmesh.ComputeFaceNormals(); //normal to faces
    globalmesh.ComputeVertexNormals(); //normals to vertex

    //construct colors from KS
    timer.Reset();
    timer.Start();
    hkscomputer.SetColorsFromKS(&globalmesh);
    timer.Stop();
    cout<<"Constructing colors from KS :"<< timer.GetTotal()/1000.0<<" ms"<<endl;
    //ok now here render
    id_globalmesh=glGenLists(1);
    glNewList(id_globalmesh,GL_COMPILE_AND_EXECUTE);

    //-----Play area
    int const SIZE=500;
    float inc = floor(globalmesh.vertices.size()/SIZE);

    int tc=0;
    for (uint i=0; tc<SIZE; i+=inc){
        tc++;
        Vector3d v1(globalmesh.vertices[i]);
        glColor3f(1,1,0);
        glBegin(GL_POINTS);
        glVertex3f(v1[0],v1[1],v1[2]);
        glEnd();
    }

    timer.Reset(); timer.Start();
    ShapeContext shapedesc;
    int type;
    //Find Full SCD Matrix, and classify the mesh
    type=shapedesc.FillSC(&globalmesh,&hkscomputer);
    timer.Stop();
    cout<<"Shape Context computation time :"<< timer.GetTotal()/1000.0/60/60<<" mins"<<endl;
    switch(type){
        case HUMAN: PrintMessage( 500, 200, "This is a Human!" );
        break;
        case BEAR: PrintMessage( 500, 200, "This is a Bear!" );
        break;
        case ANT: PrintMessage( 500, 200, "This is an Ant!" );
        break;
        default: PrintMessage( 500, 200, "Ooops, I broke up, execute me again, I will work now!" );
    }

    //DRAW SHAPE CONTEXT FOR A POINT AREA
   /* globalmesh.BuildDistanceLabels(5);
    vector<int> z1 = hkscomputer.Between(&globalmesh, 20, 30, 0.0);
    for (uint i=0; i<z1.size(); i++){
        Vector3d v1(globalmesh.vertices[z1[i]]);
        glColor3f(1,1,0);
        glBegin(GL_POINTS);
        glVertex3f(v1[0],v1[1],v1[2]);
        glEnd();
    }
    vector<int> z2 = hkscomputer.Between(&globalmesh, 10, 20, 0.0);
    for (uint i=0; i<z2.size(); i++){
        Vector3d v1(globalmesh.vertices[z2[i]]);
        glColor3f(1,0,0);
        glBegin(GL_POINTS);
        glVertex3f(v1[0],v1[1],v1[2]);
        glEnd();
    }
    vector<int> z3 = hkscomputer.Between(&globalmesh, 0, 10, 0.0);
    for (uint i=0; i<z3.size(); i++){
        Vector3d v1(globalmesh.vertices[z3[i]]);
        glColor3f(1,1,1);
        glBegin(GL_POINTS);
        glVertex3f(v1[0],v1[1],v1[2]);
        glEnd();
    }*/
    //END TEST AREA


    //--------
    glEnable(GL_LIGHTING);
    globalmesh.Draw(VERTEX_NORMAL_RGB);


    glEndList();

    //now render
    glutMainLoop();

	return 0;

}

