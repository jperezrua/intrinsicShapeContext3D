/***************************************************************************
                                  ShapeContext.cpp
								-------------------
 ***************************************************************************/
 
#include "ShapeContext.h"


ShapeContext::ShapeContext(){
    this->MaxR=20;
    this->MedR=10;
    this->MinR=5;
    this->SC=MatrixXf::Zero(PTS,12);
    this->SCnorm=VectorXf::Zero(PTS);
}

ShapeContext::~ShapeContext(){
}

void ShapeContext::SetPoints(NeighborMesh *globalmesh){
    float inc = floor(globalmesh->vertices.size()/PTS);
    this->centerpts.clear();
    int tc=0;
    for (uint i=0; tc<PTS; i+=inc){
        tc++;
        this->centerpts.push_back(i);
    }
}

int ShapeContext::FillSC(NeighborMesh *globalmesh, GenericKS *hkscomputer){

    char nfn[255];
    sprintf(nfn, "%s.scd", hkscomputer->name);
    std::ifstream ifile(nfn);

    if ( !ifile.is_open() ){
        cout<<"Computing SCD."<<endl;

        this->SetPoints(globalmesh);
        for (uint p=0;p<PTS;p++){
            int P=this->centerpts[p]; //Go solving point by point in the vector centerpts (resampled mesh)

            vector<int> externpts; //12 points centered in P
            globalmesh->BuildDistanceLabels(P); //Distance labels from point P

            vector<int> ref = hkscomputer->CloserTo(globalmesh, P, this->MaxR,1);//Select every points
                                                                                    //centered on P, and in its outter ring
            int maxindex=0;
            int eqindex=0;
            if (ref.size()<4){
                std::cout<<"ref.size()<4!"<<std::endl;
                int tcont=0;
                for (uint j=0;j<ref.size();j++){
                    tcont++;
                    externpts.push_back(ref[j]);
                }
                for (int j=0;j<4-tcont;j++){
                    externpts.push_back(P);
                }
            }else{
                //std::cout<<"Point "<<i<<std::endl;
                /** COMPUTE 4 POINTS OF EXTERNAL RING **/

                //std::cout<<"ref.size(): "<<ref.size()<<std::endl;
                //Find Point 0 and 1

                externpts.push_back(ref[0]);
                for (uint i=1;i<ref.size();i++){
                    if (hkscomputer->GeodesicDistance(globalmesh,ref[0], ref[i]) >
                        hkscomputer->GeodesicDistance(globalmesh,ref[0], ref[maxindex])){
                        maxindex=i;
                    }
                }externpts.push_back(ref[maxindex]);

                //Find Point 2
                for (uint i=1;i<ref.size();i++){
                    float A=hkscomputer->GeodesicDistance(globalmesh,externpts[0], ref[i]);
                    float B=hkscomputer->GeodesicDistance(globalmesh,externpts[1], ref[i]);
                    float C=hkscomputer->GeodesicDistance(globalmesh,externpts[0], ref[eqindex]);
                    float D=hkscomputer->GeodesicDistance(globalmesh,externpts[1], ref[eqindex]);
                    if ( abs(A-B) < abs(C-D)){
                        eqindex=i;
                    }
                }externpts.push_back(ref[eqindex]);

                //Find Point 3
                maxindex=eqindex;
                for (uint i=0;i<ref.size();i++){
                    if (hkscomputer->GeodesicDistance(globalmesh,ref[eqindex], ref[i]) >
                        hkscomputer->GeodesicDistance(globalmesh,ref[eqindex], ref[maxindex])){
                        maxindex=i;
                    }
                }externpts.push_back(ref[maxindex]);
            }

            /** COMPUTE 4 POINTS OF THE MIDDLE RING **/
            //add 4 points centered in P and in the Middle Ring of P to vector externpts
            //Distance labels from point P are still in mesh->Labels
            vector<int> ref2 = hkscomputer->CloserTo(globalmesh, P, this->MedR);//Select every points
                                                                                //centered on P, and in its middle ring
            if (ref2.size()<4){
                std::cout<<"ref2.size()<4!"<<std::endl;
                int tcont=0;
                for (uint j=0;j<ref2.size();j++){
                    tcont++;
                    externpts.push_back(ref2[j]);
                }
                for (int j=0;j<4-tcont;j++){
                    externpts.push_back(P);
                }
            }else{
                //std::cout<<"ref2.size(): "<<ref2.size()<<std::endl;
                //Find Point 4 and 5
                maxindex=0;
                externpts.push_back(hkscomputer->FindCloserToPoint(globalmesh,externpts[0],ref2)); //Point4 is the closer in ref2 to Point 0
                for (uint i=1;i<ref2.size();i++){
                    if (hkscomputer->GeodesicDistance(globalmesh,externpts[4], ref2[i]) >
                        hkscomputer->GeodesicDistance(globalmesh,externpts[4], ref2[maxindex])){ //Point5 is the fartest from Point4
                        maxindex=i;
                    }
                }externpts.push_back(ref2[maxindex]);

                //Find Point 6
                eqindex=0;
                for (uint i=1;i<ref2.size();i++){
                    float A=hkscomputer->GeodesicDistance(globalmesh,externpts[4], ref2[i]);
                    float B=hkscomputer->GeodesicDistance(globalmesh,externpts[5], ref2[i]);
                    float C=hkscomputer->GeodesicDistance(globalmesh,externpts[4], ref2[eqindex]);
                    float D=hkscomputer->GeodesicDistance(globalmesh,externpts[5], ref2[eqindex]);
                    if ( abs(A-B) < abs(C-D)){
                        eqindex=i;
                    }
                }externpts.push_back(ref2[eqindex]);

                //Find Point 7
                maxindex=eqindex;
                for (uint i=0;i<ref2.size();i++){
                    if (hkscomputer->GeodesicDistance(globalmesh,ref2[eqindex], ref2[i]) >
                        hkscomputer->GeodesicDistance(globalmesh,ref2[eqindex], ref2[maxindex])){
                        maxindex=i;
                    }
                }externpts.push_back(ref2[maxindex]);
            }
            /** COMPUTE 4 POINTS OF THE INNER RING **/
            //add 4 pmaria, jill is too shyoints centered in P and in the Inner Ring of P to vector externpts
            //Distance labels from point P are still in mesh->Labels
            vector<int> ref3 = hkscomputer->CloserTo(globalmesh, P, this->MinR);//Select every points
                                                                                //centered on P, and in its inner ring
            if (ref3.size()<4){
                std::cout<<"ref3.size()<4!"<<std::endl;
                int tcont=0;
                for (uint j=0;j<ref3.size();j++){
                    tcont++;
                    externpts.push_back(ref3[j]);
                }
                for (int j=0;j<4-tcont;j++){
                    externpts.push_back(P);
                }
            }else{
                //std::cout<<"extpoints.size() should be 8: "<<externpts.size()<<std::endl;
                //Find Point 8 and 9
                maxindex=0;
                //Point 8 is the closer in ref3 to Point4
                externpts.push_back(hkscomputer->FindCloserToPoint(globalmesh,externpts[4],ref3));
                //std::cout<<"ref3.size(): "<<ref3.size()<<std::endl;
                for (uint i=1;i<ref3.size();i++){
                    if (hkscomputer->GeodesicDistance(globalmesh,externpts[8], ref3[i]) >
                        hkscomputer->GeodesicDistance(globalmesh,externpts[8], ref3[maxindex])){ //P9 is fartest to P8
                        maxindex=i;
                    }
                }externpts.push_back(ref3[maxindex]);

                //Find Point 10
                eqindex=0;
                for (uint i=1;i<ref3.size();i++){
                    float A=hkscomputer->GeodesicDistance(globalmesh,externpts[0], ref3[i]);
                    float B=hkscomputer->GeodesicDistance(globalmesh,externpts[1], ref3[i]);
                    float C=hkscomputer->GeodesicDistance(globalmesh,externpts[0], ref3[eqindex]);
                    float D=hkscomputer->GeodesicDistance(globalmesh,externpts[1], ref3[eqindex]);
                    if ( abs(A-B) < abs(C-D)){
                        eqindex=i;
                    }
                }externpts.push_back(ref3[eqindex]);

                //Find Point 11
                maxindex=eqindex;
                for (uint i=0;i<ref3.size();i++){
                    if (hkscomputer->GeodesicDistance(globalmesh,ref3[eqindex], ref3[i]) >
                        hkscomputer->GeodesicDistance(globalmesh,ref3[eqindex], ref3[maxindex])){
                        maxindex=i;
                    }
                }externpts.push_back(ref3[maxindex]);
            }
            /** I HAVE THE POINTS, NOW START FILLING THE SC **/
            vector <int> pointsOutter, pointsMiddle, pointsInner;
            pointsOutter=hkscomputer->Between(globalmesh, this->MedR, this->MaxR);
            pointsMiddle=hkscomputer->Between(globalmesh, this->MinR, this->MedR);
            pointsInner =hkscomputer->Between(globalmesh, 0, this->MinR);

            int contbin0=0,contbin1=0,contbin2=0,contbin3=0,contbin4=0,contbin5=0,contbin6=0;
            int contbin11=0,contbin10=0,contbin9=0,contbin8=0,contbin7=0;


            for (uint k=0;k<pointsInner.size();k++){
                float dto0=hkscomputer->GeodesicDistance(globalmesh,pointsInner[k],externpts[0]);
                float dto1=hkscomputer->GeodesicDistance(globalmesh,pointsInner[k],externpts[1]);
                float dto2=hkscomputer->GeodesicDistance(globalmesh,pointsInner[k],externpts[2]);
                float dto3=hkscomputer->GeodesicDistance(globalmesh,pointsInner[k],externpts[3]);

                if( dto0<dto1 && dto0<dto2 && dto0<dto3  ){
                    contbin0++;
                    SC(p,0)+=hkscomputer->KS(pointsInner[k]); //Add the value of the KS for that point
                }
                if( dto1<dto0 && dto1<dto2 && dto1<dto3  ){
                    contbin1++;
                    SC(p,1)+=hkscomputer->KS(pointsInner[k]); //Add the value of the KS for that point
                }
                if( dto2<dto1 && dto2<dto0 && dto2<dto3  ){
                    contbin2++;
                    SC(p,2)+=hkscomputer->KS(pointsInner[k]); //Add the value of the KS for that point
                }
                if( dto3<dto1 && dto3<dto2 && dto3<dto0  ){
                    contbin3++;
                    SC(p,3)+=hkscomputer->KS(pointsInner[k]); //Add the value of the KS for that point
                }
            }
            for (uint k=0;k<pointsMiddle.size();k++){
                float dto4=hkscomputer->GeodesicDistance(globalmesh,pointsMiddle[k],externpts[4]);
                float dto5=hkscomputer->GeodesicDistance(globalmesh,pointsMiddle[k],externpts[5]);
                float dto6=hkscomputer->GeodesicDistance(globalmesh,pointsMiddle[k],externpts[6]);
                float dto7=hkscomputer->GeodesicDistance(globalmesh,pointsMiddle[k],externpts[7]);
                if( dto4<dto5 && dto4<dto6 && dto4<dto7  ){
                    contbin4++;
                    SC(p,4)+=hkscomputer->KS(pointsMiddle[k]); //Add the value of the KS for that point
                }
                if( dto5<dto4 && dto5<dto6 && dto5<dto7  ){
                    contbin5++;
                    SC(p,5)+=hkscomputer->KS(pointsMiddle[k]); //Add the value of the KS for that point
                }
                if( dto6<dto4 && dto6<dto5 && dto6<dto7  ){
                    contbin6++;
                    SC(p,6)+=hkscomputer->KS(pointsMiddle[k]); //Add the value of the KS for that point
                }
                if( dto7<dto4 && dto7<dto5 && dto7<dto6  ){
                    contbin7++;
                    SC(p,3)+=hkscomputer->KS(pointsMiddle[k]); //Add the value of the KS for that point
                }
            }
            for (uint k=0;k<pointsOutter.size();k++){
                float dto8=hkscomputer->GeodesicDistance(globalmesh,pointsOutter[k],externpts[8]);
                float dto9=hkscomputer->GeodesicDistance(globalmesh,pointsOutter[k],externpts[9]);
                float dto10=hkscomputer->GeodesicDistance(globalmesh,pointsOutter[k],externpts[10]);
                float dto11=hkscomputer->GeodesicDistance(globalmesh,pointsOutter[k],externpts[11]);
                if( dto8<dto9 && dto8<dto10 && dto8<dto11  ){
                    contbin8++;
                    SC(p,8)+=hkscomputer->KS(pointsOutter[k]); //Add the value of the KS for that point
                }
                if( dto9<dto8 && dto9<dto10 && dto9<dto11  ){
                    contbin9++;
                    SC(p,9)+=hkscomputer->KS(pointsOutter[k]); //Add the value of the KS for that point
                }
                if( dto10<dto8 && dto10<dto9 && dto10<dto11  ){
                    contbin10++;
                    SC(p,10)+=hkscomputer->KS(pointsOutter[k]); //Add the value of the KS for that point
                }
                if( dto11<dto8 && dto11<dto9 && dto11<dto10  ){
                    contbin11++;
                    SC(p,11)+=hkscomputer->KS(pointsOutter[k]); //Add the value of the KS for that point
                }
            }
            if(contbin0!=0) SC(p,0)=SC(p,0)/contbin0; if(contbin6!=0) SC(p,6)=SC(p,6)/contbin6;
            if(contbin1!=0) SC(p,1)=SC(p,1)/contbin1; if(contbin7!=0) SC(p,7)=SC(p,7)/contbin7;
            if(contbin2!=0) SC(p,2)=SC(p,2)/contbin2; if(contbin8!=0) SC(p,8)=SC(p,8)/contbin8;
            if(contbin3!=0) SC(p,3)=SC(p,3)/contbin3; if(contbin9!=0) SC(p,9)=SC(p,9)/contbin9;
            if(contbin4!=0) SC(p,4)=SC(p,4)/contbin4; if(contbin10!=0) SC(p,10)=SC(p,10)/contbin10;
            if(contbin5!=0) SC(p,5)=SC(p,5)/contbin5; if(contbin11!=0) SC(p,11)=SC(p,11)/contbin11;
        }
        ifile.close();
        std::ofstream ofile(nfn);
        for (uint i=0;i<PTS;i++){
            for (uint j=0;j<12;j++){
                char towrite[100];
                sprintf(towrite, "%f,",SC(i,j));
                ofile<<towrite<<endl;
            }
        }
        ofile.close();
    }else{
        char toread[100]="";
        for (uint i=0;i<PTS;i++){
            for (uint l=0;l<12;l++){
                char item='a'; int j=0;
                ifile>>item;
                while(item!=','){
                    toread[j]=item;
                    ifile>>item;
                    j++;
                }toread[j]='\0';
                float t;
                sscanf(toread, "%f", &t);
                SC(i,l) = t;
            }
        }ifile.close();

        cout<<"DFT starting..."<<endl;
        cv::Mat CompMat = cv::Mat::zeros(PTS, 12, CV_32F);

        for (uint i=0;i<PTS;i++){
            vector< complex<double> > data;
            vector< complex<double> > dft;
            vector< double > absdft;
            for (uint j=0;j<12;j++)
                data.push_back( SC(i,j) );
            dft = DFT(data);
            absdft = ABS(dft);
            for (uint j=0;j<12;j++)
                CompMat.at<float>(i,j)=absdft[j];
        }
        cout<<"DFT finished..."<<endl;

        /** CompMat is the FFT of the current mesh, Now load the templates 1 by 1 **/

        //cv::Mat Temp = cv::Mat::zeros(12, PTS, CV_32F);
        //cv::flip(CompMat, Temp, 1);
        cv::imshow("Mesh ABS(FFT(SCD))",CompMat.t());
        cv::waitKey(100);

        /*** 1. HUMAN ***/
        std::ifstream ifileh("../../Resampled/0001.null.0.ply.scd");
        cv::Mat TemplateHolder(PTS, 12, CV_32F);
        cv::Mat ResHolder(1, 1, CV_32F);
        float matchhuman;
        if ( !ifileh.is_open() ) cout<<"file not read"<<endl;
        cout<<"Starting HUMAN template extraction"<<endl;
        for (uint i=0;i<PTS;i++){
            for (uint l=0;l<12;l++){
                char item='a'; int j=0;
                ifileh>>item;
                while(item!=','){
                    toread[j]=item;
                    ifileh>>item;
                    j++;
                }toread[j]='\0';
                float t;
                sscanf(toread, "%f", &t);
                SC(i,l) = t;
            }
        }ifileh.close();
        cout<<"HUMAN template read"<<endl;
        for (uint i=0;i<PTS;i++){
            vector< complex<double> > data;
            vector< complex<double> > dft;
            vector< double > absdft;
            for (uint j=0;j<12;j++)
                data.push_back( SC(i,j) );
            dft = DFT(data);
            absdft = ABS(dft);
            for (uint j=0;j<12;j++)
                TemplateHolder.at<float>(i,j)=absdft[j];
        }
        cout<<"HUMAN template FFT ready"<<endl;
        cv::matchTemplate( CompMat, TemplateHolder, ResHolder, CV_TM_CCORR_NORMED );
        matchhuman = ResHolder.at<float>(0,0);

        /*** BEAR ***/
        std::ifstream ifileb("../../Resampled/bear1.ply.scd");
        float matchbear;

        for (uint i=0;i<PTS;i++){
            for (uint l=0;l<12;l++){
                char item='a'; int j=0;
                ifileb>>item;
                while(item!=','){
                    toread[j]=item;
                    ifileb>>item;
                    j++;
                }toread[j]='\0';
                float t;
                sscanf(toread, "%f", &t);
                SC(i,l) = t;
            }
        }ifileb.close();
        for (uint i=0;i<PTS;i++){
            vector< complex<double> > data;
            vector< complex<double> > dft;
            vector< double > absdft;
            for (uint j=0;j<12;j++)
                data.push_back( SC(i,j) );
            dft = DFT(data);
            absdft = ABS(dft);
            for (uint j=0;j<12;j++)
                TemplateHolder.at<float>(i,j)=absdft[j];
        }
        cv::matchTemplate( CompMat, TemplateHolder, ResHolder, CV_TM_CCORR_NORMED );
        matchbear = ResHolder.at<float>(0,0);

        /*** ANT ***/
        std::ifstream ifilea("../../Resampled/ant1.ply.scd");
        float matchant;

        for (uint i=0;i<PTS;i++){
            for (uint l=0;l<12;l++){
                char item='a'; int j=0;
                ifilea>>item;
                while(item!=','){
                    toread[j]=item;
                    ifilea>>item;
                    j++;
                }toread[j]='\0';
                float t;
                sscanf(toread, "%f", &t);
                SC(i,l) = t;
            }
        }ifilea.close();
        for (uint i=0;i<PTS;i++){
            vector< complex<double> > data;
            vector< complex<double> > dft;
            vector< double > absdft;
            for (uint j=0;j<12;j++)
                data.push_back( SC(i,j) );
            dft = DFT(data);
            absdft = ABS(dft);
            for (uint j=0;j<12;j++)
                TemplateHolder.at<float>(i,j)=absdft[j];
        }
        cv::matchTemplate( CompMat, TemplateHolder, ResHolder, CV_TM_CCORR_NORMED );
        matchant = ResHolder.at<float>(0,0);

        int retval=-1;
        if (matchhuman>=matchant && matchhuman>=matchbear){
            retval = HUMAN;
        }else if (matchbear>matchhuman && matchbear>matchant){
            retval = BEAR;
        }else{
            retval = ANT;
        }
        return retval;
    }
    return -1;
}

vector< complex<double> > ShapeContext::DFT(vector< complex<double> >& Data){
    const int TAM = Data.size();
    vector< complex<double> > out(TAM, 0);
    for(int i=0;i<TAM; i++){
        out[i] = complex<double>(0.0, 0.0);
        for(int j=0; (j < TAM); j++)
            out[i] += Data[j] * polar<double>(1.0, - 2 * PI * i * j / TAM);
    }
    return out;
}

vector< double > ShapeContext::ABS(vector< complex<double> > &Data){
    const int TAM = Data.size();
    vector< double > out(TAM, 0);
    for(int i=0; i < TAM; i++)
        out[i]=abs(Data[i]);
    return out;
}


