/***************************************************************************
                                 useful.cpp
                             -------------------
 ***************************************************************************/


#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <GL/glut.h>
#include <cstring>

#include "useful.h"
#include "Constante.h"
#include "scene.h"
using namespace std;

Vector3d DoubleToColor( const double d ){

	if(d<0) return Vector3d(0,0,0);
	if(0<=d && d<0.25)		{ return Vector3d(0 , d*4.0 ,1);}

	if(0.25<=d && d<0.5)	{ return Vector3d(0 , 1.0 , 2.0-4.0*d);}

	if(0.5<=d && d<0.75)	{ return Vector3d(4.0*d - 2.0 , 1.0   ,0);	}

	if(0.75<=d && d<=1.0)	{ return Vector3d(1.0 , 4.0-4.0*d   ,0);}

	return Vector3d(1,1,1);

 }

double ColorToDouble( Vector3d RGB){

	int key = (int)(RGB[0]>0)*4 + (int)(RGB[1]>0)*2 + (int)(RGB[2]>0);
	double t;
	switch (key){

		case 1 : t = 0.0;	break;			//ok	--> (0,0,1)
		case 2 : t = 0.5;	break;			//ok	-->	(0,1,0)
		case 4 : t = 1;		break;			//ok	--> (1,0,0)
		case 3 : {

			if(RGB[1] == 1) t =	0.5 - 0.25 * RGB[2];	//ok	-->	(0,1,X)
			else 			t = RGB[1] * 0.25;			//ok	--> (0,X,1)
		}	break;

		case 6 : {

			if(RGB[1] == 1) t = RGB[0] * 0.25 + 0.5;	//ok	--> (X,1,0)
			else 			t=	1 - 0.25 * RGB[1];		//ok	--> (1,X,0)
		} break;
		default : return 0;
	}
	return t;
}

double sign(double x)	{

	if(x<0) return -1.0;
	else return 1.0;
}

void AddGaussianNoise(vector<Vector3d> & vertices, vector<Vector3d> normals, double sigma,double mu)
{
	if(vertices.size()==0 || normals.size()==0 || vertices.size() != normals.size()){cout<<"AddGaussianNoise: array size issue"<<endl;exit(0);}

	// Seed the random-number generator with current time so that
	// the numbers will be different every time we run.

	srand( (unsigned)time( NULL ) );

	double x, u1, u2;

	for(unsigned int i=0; i<vertices.size(); i++){
		//generate u1 and u2 supposedly independent random variables in [0,1] (uniform distribution)

		 u1= rand()/(double)(RAND_MAX);
		 u2= (rand()+1)/((double)RAND_MAX+1);	//+1 to avoid log(0) in next op

		// generate gaussian variable for mu=0, sig =1

		 x = cos(2*PI*u1)*sqrt(-2*log(u2));

		// v.a. x follows a gaussian distrib' Gss(mu,sig)

		 x = mu + sigma*x;

		//update vertex along normal

		vertices[i] += x*normals[i];
	}
}


//--
//
// StringToBinding
//
//--

// Convert a string to an attribute binding
static AttributeBinding StringToBinding( const string& s )
{
	if( s == "PER_VERTEX_INDEXED" )
	{
		return PER_VERTEX_BINDING;
	}
	if( s == "PER_FACE_INDEXED" )
	{
		return PER_FACE_BINDING;
	}
	return UNKNOWN_BINDING;
}

//--
//
// BindingToString
//
//--
// Convert an attribute binding to a string
static string BindingToString( const AttributeBinding& ab )
{
	if( ab == PER_VERTEX_BINDING )
	{
		return "PER_VERTEX_INDEXED";
	}
	if( ab == PER_FACE_BINDING )
	{
		return "PER_FACE_INDEXED";
	}
	return "UNKNOWN_BINDING";
}

//Split
unsigned int mySplit(const std::string &txt, std::vector<std::string> &strs, char ch){ //stupid function
    unsigned int pos = txt.find( ch );
    unsigned int initialPos = 0;
    strs.clear();
    unsigned int sz=txt.size();

    while( pos != std::string::npos && initialPos<sz) {
        strs.push_back( txt.substr( initialPos, pos - initialPos + 1 ) );
        initialPos = pos + 1;
        pos = txt.find( ch, initialPos );
    }

    strs.push_back( txt.substr( initialPos, std::min( pos, sz ) - initialPos + 1 ) );


    return strs.size();
}

//--
//
// ReadOff : for reading meshes PLY format. Not color, not textures, just faces and vertex
//
//--
bool ReadOff( Mesh& mesh, const string& file_name ){
	cout<<"Reading OFF File"<<endl;     //
	ifstream file;                      // File to read
	file.open( file_name.c_str() ); 	// Open file

	if( !file.is_open() ){              // File open?
		cout<<"NO OFF FILE..."<<endl;
		return false;
	}else{                              // Is Open
        size_t pos;                     // String position
        string line;                    // Lines to process

        getline( file, line );          // Get first line of the file
        line = UpperCase(line);
        pos = line.find("OFF");         // Is this a ply formatted file? Seek for the KEY(ply)
        if( pos > line.length() ) {     //Key is not here
            cout<<"OFF FILE BAD FORMATTED..."<<endl;
            return false;
        }else{                                  //KEY is here
                getline( file, line );          // Get 2nd line of the file

                mesh.ClearAll();                                    // Finally, start messing around with the mesh


                Vector3d vec3d;                                 // Temp vector 3D (vertex, color, normal)
                Vector3i vec3i;                                 // Temp vector 3I (face)
                vector<string> readvals;                        // Read elements in a line
                int cvertex=0, cfaces=0;                        // counters

                while( getline( file, line ) ){                     // Now, read every line!
                    line=" "+line+" ";

                    mySplit(line, readvals, ' ');                   // Put separated vals in a vector

                    if( readvals.size()==5 ){
                        for (unsigned int i=1;i<readvals.size()-1;i++){
                            vec3d[i-1] = atof( (readvals[i]).c_str() );
                            //cout<<"vec3d["<<i-1<<"]="<<vec3d[i-1]<<endl;
                        }//cout<<endl;
                        mesh.AddVertex( vec3d );                    //New vertex
                        cvertex++;
                    }
                    if( readvals.size()==6){                       //This 6 is magic
                        for (unsigned int i=1;i<readvals.size()-2;i++){
                             vec3i[i-1] = atoi( (readvals[i+1]).c_str() )-1;
                             //cout<<"vec3i["<<i-1<<"]="<<vec3i[i-1]<<endl;
                        }//cout<<endl;
                        mesh.AddFace( vec3i );                      //New Face
                        cfaces++;
                    }
                }
                cout<<"Total vertices: "<<cvertex<<endl;
                cout<<"Total faces: "<<cfaces<<endl;
            }
            AttributeBinding normal_binding(PER_VERTEX_BINDING); // Temp normal binding
            AttributeBinding texture_binding(PER_VERTEX_BINDING); // Temp texture binding

            // Check vertex normals
            if( (normal_binding!=PER_VERTEX_BINDING) || (mesh.VertexNormalNumber()!=mesh.VertexNumber()) )
                mesh.ClearVertexNormals();

            // Normalize normals
            if( mesh.VertexNormalNumber() != 0 ) for( int i=0; i<mesh.VertexNormalNumber(); i++ ) mesh.VertexNormal(i).normalized();

            // Check texture coordinates
            if( (texture_binding!=PER_VERTEX_BINDING) || (mesh.TextureNumber()!=mesh.VertexNumber()) ){
                mesh.ClearTextures();
                mesh.ClearTextureName();
            }

            // Check colors
            if( (mesh.ColorBinding()==PER_FACE_BINDING) && (mesh.ColorNumber()!=mesh.FaceNumber()) ){
                mesh.ClearColors();
                mesh.ColorBinding() = UNKNOWN_BINDING;
            }else if( (mesh.ColorBinding()==PER_VERTEX_BINDING) && (mesh.ColorNumber()!=mesh.VertexNumber()) ){
                mesh.ClearColors();
                mesh.ColorBinding() = UNKNOWN_BINDING;
            }
        }
        file.close();
        return true;
}


//--
//
// ReadPly : for reading meshes PLY format. Not color, not textures, just faces and vertex
//
//--
bool ReadPly( Mesh& mesh, const string& file_name ){
	cout<<"Reading PLY File"<<endl;     //
	ifstream file;                      // File to read
	file.open( file_name.c_str() ); 	// Open file

	if( !file.is_open() ){              // File open?
		cout<<"NO PLY FILE..."<<endl;
		return false;
	}else{                              // Is Open
        size_t pos;                     // String position
        string line;                    // Lines to process

        getline( file, line );          // Get first line of the file
        pos = line.find("ply");         // Is this a ply formatted file? Seek for the KEY(ply)
        if( pos > line.length() ) {     //Key is not here
            cout<<"PLY FILE BAD FORMATTED..."<<endl;
            return false;
        }else{                          //KEY is here
            getline( file, line );      //Get 2nd line of the file
            pos = line.find("ascii");   // Is this ascii formatted file? Seek for the KEY(ascii)
            if( pos > line.length() ) {                             // Key is not here
                cout<<"PLY IS NOT ASCII..."<<endl;
                return false;
            }else{                                                  // ply, ascii, this is the right way
                cout<<"Good PLY..."<<endl;
                mesh.ClearAll();                                    // Finally, start messing around with the mesh
                do{
                    getline( file, line );
                    pos = line.find("end_header");
                }while( pos > line.length() ); // Increase line position until find end_header

                Vector3d vec3d;                                 // Temp vector 3D (vertex, color, normal)
                Vector3i vec3i;                                 // Temp vector 3I (face)
                vector<string> readvals;                        // Read elements in a line
                int cvertex=0, cfaces=0;                        // counters
                while( getline( file, line ) ){                     // Now, read every line!
                    line=" "+line;
                    mySplit(line, readvals, ' ');                   // Put separated vals in a vector


                    if( readvals.size()==5 ){
                        for (unsigned int i=1;i<readvals.size()-1;i++){
                            vec3d[i-1] = atof( (readvals[i]).c_str() );
                            //cout<<"vec3d["<<i-1<<"]="<<vec3d[i-1]<<endl;
                        }//cout<<endl;
                        cvertex++;
                        mesh.AddVertex( vec3d );                    //New vertex
                    }
                    if( readvals.size()==6 ){                       //This 6 is magic
                        for (unsigned int i=1;i<readvals.size()-2;i++){
                             vec3i[i-1] = atoi( (readvals[i+1]).c_str() );
                             //cout<<"vec3i["<<i-1<<"]="<<vec3i[i-1]<<endl;
                        }//cout<<endl;
                        cfaces++;
                        mesh.AddFace( vec3i );                      //New Face
                    }
                }
                cout<<"Total vertices: "<<cvertex<<endl;
                cout<<"Total faces: "<<cfaces<<endl;
            }
            AttributeBinding normal_binding(PER_VERTEX_BINDING); // Temp normal binding
            AttributeBinding texture_binding(PER_VERTEX_BINDING); // Temp texture binding

            // Check vertex normals
            if( (normal_binding!=PER_VERTEX_BINDING) || (mesh.VertexNormalNumber()!=mesh.VertexNumber()) )
                mesh.ClearVertexNormals();

            // Normalize normals
            if( mesh.VertexNormalNumber() != 0 ) for( int i=0; i<mesh.VertexNormalNumber(); i++ ) mesh.VertexNormal(i).normalized();

            // Check texture coordinates
            if( (texture_binding!=PER_VERTEX_BINDING) || (mesh.TextureNumber()!=mesh.VertexNumber()) ){
                mesh.ClearTextures();
                mesh.ClearTextureName();
            }

            // Check colors
            if( (mesh.ColorBinding()==PER_FACE_BINDING) && (mesh.ColorNumber()!=mesh.FaceNumber()) ){
                mesh.ClearColors();
                mesh.ColorBinding() = UNKNOWN_BINDING;
            }else if( (mesh.ColorBinding()==PER_VERTEX_BINDING) && (mesh.ColorNumber()!=mesh.VertexNumber()) ){
                mesh.ClearColors();
                mesh.ColorBinding() = UNKNOWN_BINDING;
            }
        }
        file.close();
        return true;
	}
}

// Read OpenInventor/VRML1.0 file
bool ReadIv( Mesh& mesh, const string& file_name ) {

	cout<<"Reading IV/VRML"<<endl;
	ifstream file; // File to read
	string line, new_line; // Lines to process
	string word, previous_word; // Words to process
	string::iterator it; // Iterator for lines
	size_t pos; // String position
	int nlbrack(0), nrbrack(0); // Left/Right delimiter counter
	int level(0); // Node level
	int ixyz(0); // Coordinate index
	Vector3d vec3d; // Temp vector 3D (vertex, color, normal)
	Vector3i vec3i; // Temp vector 3I (face)
	Vector2d vec2d; // Temp vector 2D (texture coordinate)
	AttributeBinding normal_binding(PER_VERTEX_BINDING); // Temp normal binding
	AttributeBinding texture_binding(PER_VERTEX_BINDING); // Temp texture binding
	vector<string> node(10); // nodes - 10 levels by default

	// Open file
	file.open( file_name.c_str() );

	// Test if file is open
	if( file.is_open() == false )
	{
		cout<<"No file found"<<endl;
		return false;
	}

	//--
	// Read file header
	//--

	// Get first line of the file
	getline( file, line );

	// Try to find #Inventor in the line
	pos = line.find("#Inventor");
	if( pos > line.length() )
	{
		// Else try to find #VRML V1.0 in the line
		pos = line.find("#VRML V1.0");
		if( pos > line.length() )
		{
			cout<<"pas VRML"<<endl;
			return false;
		}
	}

	// Verify file is ascii format
	pos = line.find("ascii");
	if( pos > line.length() )
	{
		return false;
	}

	// Initialize the mesh
	mesh.ClearAll();

	//--
	// Read the file until the end reading line by line
	//--
	while( getline( file, line ) )
	{
		// Replace all comas by space
		replace( line.begin(), line.end(), ',', ' ' );

		// Initialize new_line
		new_line.erase( new_line.begin(), new_line.end() );

		//--
		// Buffer brackets and braces by space and copy in new_line
		//--
		it = line.begin();
		while( it != line.end() )
		{

			if( ((*it)=='{') || ((*it)=='}') || ((*it)=='[') || ((*it)==']') || ((*it)=='#') )
			{
				new_line += ' ';
				new_line += *it++;
				new_line += ' ';
			}
			else
			{
		   		new_line += *it++;
 			}
		}

		//--
		// Analyse each word in the line
		//--
		istringstream word_stream( new_line );
		while( word_stream >> word )
		{
			//--
			// Left bracket or brace
			//--
			if( (word == "{") || (word == "[") )
			{
				// Increment left deliminter number
				nlbrack++;
				// Get level number
				level = nlbrack - nrbrack;
				// Save level name
				if( level > (int)node.size() )
				{
					node.push_back( UpperCase(previous_word) );
				}
				else
				{
					node[level] = UpperCase(previous_word);
				}
				// Initialize coordinate index
				ixyz = 0;
			}

			//--
			// Right bracket or brace
			//--
			else if( (word == "}") || (word == "]") )
			{
				// Increment right deliminter number
				nrbrack++;
				// Get level number
				level = nlbrack - nrbrack;
				// Sanity test
				if( level < 0 )
				{
					mesh.ClearAll();
					return false;
				}
			}

			//--
			// Comment
			//--
			else if( word == "#" )
			{
				// Save current word
				previous_word = word;
				// Next line
				break;
			}

			//--
			// COORDINATE3
			//--
			else if( node[level] == "COORDINATE3" )
			{
			}

			//--
			// INDEXEDFACESET
			//--
			else if( node[level] == "INDEXEDFACESET" )
			{
			}

			//--
			// MATERIAL
			//--
			else if( node[level] == "MATERIAL" )
			{
			}

			//--
			// MATERIALBINDING
			//--
			else if( node[level] == "MATERIALBINDING" )
			{
				if( previous_word == "value" )
				{
					mesh.ColorBinding() = StringToBinding( word );
				}
			}

			//--
			// NORMALBINDING
			//--
			else if( node[level] == "NORMALBINDING" )
			{
				if( previous_word == "value" )
				{
					normal_binding = StringToBinding( word );
   				}
			}

			//--
			// TEXTURECOORDINATEBINDING
			//--
			else if( node[level] == "TEXTURECOORDINATEBINDING" )
			{
				if( previous_word == "value" )
				{
					texture_binding = StringToBinding( word );
				}
			}

			//--
			// POINT
			//--
			else if( node[level] == "POINT" )
			{
				if( node[level-1] == "COORDINATE3" )
				{
					// Get current value
					vec3d[ixyz] = atof( word.c_str() );
					// Complete coordinate ?
					if( ixyz == 2 )
					{
						mesh.AddVertex( vec3d );
						ixyz = 0;
					}
					else
					{
						ixyz++;
					}
				}
				else if( node[level-1] == "TEXTURECOORDINATE2" )
				{
					// Get current value
					vec2d[ixyz] = atof( word.c_str() );
					// Complete coordinate ?
					if( ixyz == 1 )
					{
						mesh.AddTexture( vec2d );
						ixyz = 0;
					}
					else
					{
						ixyz++;
					}
				}
			}

			//--
			// DIFFUSECOLOR
			//--
			else if( node[level] == "DIFFUSECOLOR" )
			{
				if( node[level-1] == "MATERIAL" )
				{
					// Get current value
					vec3d[ixyz] = atof( word.c_str() );
					// Complete coordinate ?
					if( ixyz == 2 )
					{
						mesh.AddColor( vec3d );
						ixyz = 0;
					}
					else
					{
						// Next coordinate
						ixyz++;
					}
				}
			}

			//--
			// VECTOR
			//--
			else if( node[level] == "VECTOR" )
			{
				if( node[level-1] == "NORMAL" )
				{
					// Get current value
					vec3d[ixyz] = atof( word.c_str() );
					// Complete coordinate ?
					if( ixyz == 2 )
					{
						mesh.AddVertexNormal( vec3d );
						ixyz = 0;
					}
					else
					{
						// Next coordinate
						ixyz++;
					}
				}
			}

			//--
			// TEXTURE2
			//--
			else if( node[level] == "TEXTURE2" )
			{
				if( previous_word == "filename" )
				{
					// Get texture filename
					mesh.TextureName() = word;
				}
			}

			//--
			// COORDINDEX
			//--
			else if( node[level] == "COORDINDEX" )
			{
				if( node[level-1] == "INDEXEDFACESET" )
				{
					// -1 value
					if( ixyz == 3 )
					{
						// Next face
						ixyz = 0;
						continue;
					}
					// Get value
					vec3i[ixyz] = atoi( word.c_str() );
					// Complete coordinate ?
					if( ixyz == 2 )
					{
						mesh.AddFace( vec3i );
					}
					// Next coordinate
					ixyz++;
				}
			}

			//--
			// MATERIALINDEX
			//--
			else if( node[level] == "MATERIALINDEX" )
			{
				if( node[level-1] == "INDEXEDFACESET" )
				{
				}
			}

			//--
			// TEXTURECOORDINDEX
			//--
			else if( node[level] == "TEXTURECOORDINDEX" )
			{
				if( node[level-1] == "INDEXEDFACESET" )
				{
				}
			}

			//--
			// NORMALINDEX
			//--
			else if( node[level] == "NORMALINDEX" )
			{
				if( node[level-1] == "INDEXEDFACESET" )
				{
				}
			}

			// Save current word
			previous_word = word;
		}
	}

	// Check vertex normals
	if( (normal_binding!=PER_VERTEX_BINDING) || (mesh.VertexNormalNumber()!=mesh.VertexNumber()) ){
		mesh.ClearVertexNormals();
	}

	// Normalize normals
	if( mesh.VertexNormalNumber() != 0 ){
		for( int i=0; i<mesh.VertexNormalNumber(); i++ ) mesh.VertexNormal(i).normalized();
	}

	// Check texture coordinates
	if( (texture_binding!=PER_VERTEX_BINDING) || (mesh.TextureNumber()!=mesh.VertexNumber()) ){
		mesh.ClearTextures();
		mesh.ClearTextureName();
	}

	// Check colors
	if( (mesh.ColorBinding()==PER_FACE_BINDING) && (mesh.ColorNumber()!=mesh.FaceNumber()) ){
		mesh.ClearColors();
		mesh.ColorBinding() = UNKNOWN_BINDING;
	}else if( (mesh.ColorBinding()==PER_VERTEX_BINDING) && (mesh.ColorNumber()!=mesh.VertexNumber()) ){
		mesh.ClearColors();
		mesh.ColorBinding() = UNKNOWN_BINDING;
	}

	file.close();

	return true;
}

//--
//
// WriteIv
//
//--
// Write model to an OpenInventor / VRML 1.0 file

bool WriteIv( const Mesh& mesh, const string& file_name, const bool& vrml1 )
{
	int i;

	// Test if mesh is valid
	if( (mesh.VertexNumber()==0) || (mesh.FaceNumber()==0) )
	{
		cout<<"No face or no vertex"<<endl;
		return false;
	}

	// Open file for writing
	ofstream file( file_name.c_str() );

	// Test if the file is open
	if( file.is_open() == false )
	{
		cout<<"crashed opening file"<<endl;
		return false;
	}

	//--
	// Write file Header
	//--

	// VRML 1.0 file header
	if( vrml1 ) file<<"#VRML V1.0 ascii\n"<<endl;
	// or OpenInventor file header
	else file<<"#Inventor V2.1 ascii\n"<<endl;

	// Write vertex number (comment)
	file<<"# Vertices: "<<mesh.VertexNumber()<<endl;
	// Write face number (comment)
	file<<"# Faces:    "<<mesh.FaceNumber()<<endl;
	file<<endl;

	// Begin description
	file<<"Separator {"<<endl;

	//--
	// Write  vertex coordinates
	//--
	file<<"    Coordinate3 {"<<endl;
	file<<"        point ["<<endl;
	for( i=0; i<(int)mesh.VertexNumber()-1; i++ )
	{
		file<<"            "<<mesh.Vertex(i)<<","<<endl;
	}
	file<<"            "<<mesh.Vertex(mesh.VertexNumber()-1)<<endl;
	file<<"        ]"<<endl;
	file<<"    }"<<endl;

	//--
	// Write colors
	//--
	if( (mesh.ColorNumber()) && (mesh.ColorBinding()!=UNKNOWN_BINDING)  )
	{
		// Binding
		file<<"    MaterialBinding {"<<endl;
		file<<"        value "<<BindingToString(mesh.ColorBinding())<<endl;
		file<<"    }"<<endl;
		// Color
		file<<"    Material {"<<endl;
		file<<"        diffuseColor ["<<endl;
		for( i=0; i<(int)mesh.ColorNumber()-1; i++ )
		{
			file<<"            "<<mesh.Color(i)<<","<<endl;
		}
		file<<"            "<<mesh.Color(mesh.ColorNumber()-1)<<endl;
		file<<"        ]"<<endl;
		file<<"    }"<<endl;
	}

	//--
	// Write vertex normals
	//--
	if( mesh.VertexNormalNumber() == mesh.VertexNumber()  )
	{
		// Binding
		file<<"    NormalBinding {"<<endl;
		file<<"        value PER_VERTEX_INDEXED"<<endl;
		file<<"    }"<<endl;
		// Vertex normals
		file<<"    Normal {"<<endl;
		file<<"        vector ["<<endl;
		for( i=0; i<(int)mesh.VertexNormalNumber()-1; i++ )
		{
			file<<"            "<<mesh.VertexNormal(i)<<","<<endl;
		}
		file<<"            "<<mesh.VertexNormal(mesh.VertexNormalNumber()-1)<<endl;
		file<<"        ]"<<endl;
		file<<"    }"<<endl;
	}

	//--
	// Write texture
	//--
	if( (mesh.TextureNumber()==mesh.VertexNumber()) && (mesh.TextureName()!="")  )
	{
		// Texture file name
		file<<"    Texture2 {"<<endl;
		file<<"        filename \""<<mesh.TextureName()<<"\""<<endl;
		file<<"    }"<<endl;
		// Texture coordinate binding
		file<<"    TextureCoordinateBinding {"<<endl;
		file<<"        value PER_VERTEX_INDEXED"<<endl;
		file<<"    }"<<endl;
		// Texture coordinates
		file<<"    TextureCoordinate2 {"<<endl;
		file<<"        point ["<<endl;
		for( i=0; i<(int)mesh.TextureNumber()-1; i++ )
		{
			file<<"            "<<mesh.Texture(i)<<","<<endl;
		}
		file<<"            "<<mesh.Texture(mesh.TextureNumber()-1)<<endl;
		file<<"        ]"<<endl;
		file<<"    }"<<endl;
	}

	//--
	// Write face indices
	//--

	// Begin face index block
	file<<"    IndexedFaceSet  {"<<endl;

	// Index (same for vertices, colors, texture cooridnates, normals)
	file<<"        coordIndex ["<<endl;
	for( i=0; i<(int)mesh.FaceNumber()-1; i++ )
	{
		file<<"            "<<mesh.FaceIndex(i,0)<<", "<<mesh.FaceIndex(i,1)<<", "<<mesh.FaceIndex(i,2)<<", -1,"<<endl;
	}
	file<<"            "<<mesh.FaceIndex(i,0)<<", "<<mesh.FaceIndex(i,1)<<", "<<mesh.FaceIndex(i,2)<<", -1"<<endl;
	file<<"        ]"<<endl;

	// End face index block
	file<<"     }"<<endl;

	// End description
	file<<"}"<<endl;

	// Close file
	file.close();

	return true;
}



//capture current OpenGL Window and format it to a .tga file
void ScreenShot(string Fichier)
{
	unsigned char *buffer;

    GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	int w = viewport[2];//Resolution_X;
	int h = viewport[3];//Resolution_Y;

	int buf_size = 18 + (w * h * 3);	// HEADER_SIZE	==> 18

	int i;
	unsigned char temp;

	// output file
	FILE *Fichier_Screenshot_Out;
	int Nombre_Screenshot = 0;

	// file opening
	if (!(Fichier_Screenshot_Out = fopen(Fichier.c_str(), "wb")))
		return;

	Nombre_Screenshot ++;

	// mem allocation
	if (!(buffer = (unsigned char *) calloc(1, buf_size)))
		return;


	// header info
	buffer[2] = 2;			// Not compressed
	buffer[12] = w & 255;
	buffer[13] = w >> 8;
	buffer[14] = h & 255;
	buffer[15] = h >> 8;
	buffer[16] = 24;		// 24 bits per pixel

	// reading from buffer
	glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, buffer + 18);

	// Conversion  RGB to BGR
	for (i = 18; i < buf_size; i += 3)
		{
			temp = buffer[i];
			buffer[i] = buffer[i + 2];
			buffer[i + 2] = temp;
		}

	// header writing + file
	fwrite(buffer, sizeof(unsigned char), buf_size, Fichier_Screenshot_Out);

	// file closing
	fclose(Fichier_Screenshot_Out);

	// free mem
	free(buffer);
}

Vector3d DoubleToColorDiscrete( const double d, double n ){

	if(d<0) return Vector3d(0,0,0);

	//Vector3d BaseColor(0,-1,-4);
	//Vector3d Increment(1.5,2,5);

	Vector3d BaseColor(0,0,1);
	Vector3d Increment(1,1,0);

	for ( int i=0; i<n; i++)
           if(0 + (double) i/n<=d && d <= double (i+1)/n)
             { return BaseColor + /*pow(-1,i)**//*(i/n)*/Increment;
             //Vector3d(0.5-0.25*i/n , 0.75-0.1*i/n ,0.5-0.75*i/n);
             }


	return BaseColor+Increment;

 }
