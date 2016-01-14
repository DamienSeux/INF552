#include <stdexcept>
#include <vector>
#include <iostream>
#include "Droite.cpp"
#include "Homographie.cpp"
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

/* n is the number of component for the data, pt_type their type (float of int), n the min number of point for the model
 F is the class of the model, whose parameters are computed when constructor is called.

 This is the RANSAC explain in class and used in open CV and not the usual version :
 the least square fit is only applied to the best model and there is no minimum number
 of inliers.
*/
template <typename pt_type, typename dt_type, typename F,int min>
class Ransac{
private:
    bool inliers[];
    F best; // the best model so far, or null if no model has been found
    int error; // score of the best model : the number of inliers
    float thres; // thresold to decide who is an insider
    int nstep; // how many random sample we try
    vector<pair<dt_type,dt_type > > data; // the data, is points of n components of type pt_type.
    vector<int> ranks;

public :
    Ransac(const vector<pair<dt_type,dt_type> >& data,float thres,int nstep):data(data),nstep(nstep),thres(thres){
        if(data.size()<min){
            throw invalid_argument("Pas assez de points");
        }
        inliers[data.size()];
        ranks=vector<int>();
        for(int i=0;i<data.size();i++){
            ranks.push_back(i);
        }
    };

    inline void copy(bool src[],bool dst[],int size){
        for(int i=0;i<size;i++){
            dst[i]=src[i];
        }
    }

    inline void randomize(vector<int>& t){
        //sample min points at position 0..min-1 in t by swapping.
        srand((uint) time(NULL));
        if(min>t.size()){
            throw invalid_argument("Pas assez de points");
        }
        int ind;
        for(int i=0;i<min;i++){
            ind=rand() % (t.size()-i)+i;
            int temp=t[ind];
            t[ind]=t[i];
            t[i]=temp;
        }
    }

    inline void compute(){
        best=F();
        error=data.size();
        for(int i=0;i<nstep;i++){
            randomize(ranks);
            F model(data,ranks,thres);
            if(model.get_error()<error){
                best=model;
                error=model.get_error();
                //copy(model.get_inliers(),inliers,data.size());
            }
            //delete model;
        }


    }

    inline F get_best(){return best;}
    inline int get_error(){return error;}
    inline bool* get_inliers(){return inliers;}

};

/*
int main(int argc, char *argv[]){

    vector<pair<float,float> > data;
    srand((uint) time(NULL));
    for(int i=0;i<1000;i++){
        if(rand()%10!=0) {
            data.push_back(pair<float, float>(5 *(i/10), 25 * (i/10) + 12 + (rand() % (100) - 50) / (float) 10));
        }
        else{
            data.push_back(pair<float,float>(12*(i/10),14*(i/10)+78+(rand() % (100) - 50) / (float) 30));
        }
    }
    RANSAC<float,float,Droite<float>,2> test=RANSAC<float,float,Droite<float>,2>(data,0.2,10000);

    test.compute();

    float* param=test.get_best().param;
    cout<<param[0]<<","<<param[1]<<","<<param[2]<<endl;
    cout<<test.get_best().get_error();

    vector<pair<Point2f, Point2f>> data;
    float thres = 0.2;
    for(int i = 0; i<100; i++){
        for(int j = 0; j<100; j++){
            pair<float, float> p1(pow(1.1, i), pow(1.1, j));
            pair<float, float> p2(pow(1.1, i), -pow(1.1, j));
            data.push_back(pair<pair<float, float>, pair<float, float>>(p1, p2));
        }
    }

    Point2f p1(0, 0);
    Point2f p2(0.4, 0.4);
    data.push_back(pair<Point2f, Point2f>(p1, p2));
    Point2f p3(0, 1);
    Point2f p4(5.0/9.0, 5.0/9.0);
    data.push_back(pair<Point2f, Point2f>(p3, p4));
    Point2f p5(1, 0);
    Point2f p6(0.5, 0.6666);
    data.push_back(pair<Point2f, Point2f>(p5, p6));
    Point2f p7(1, 1);
    Point2f p8(0.6, 0.7);
    data.push_back(pair<Point2f, Point2f>(p7, p8));
    vector<int> rank;
    for(int i = 0; i<4; i++){
        rank.push_back(i);
    }


   // Ransac<float, Point2f, Homographie,4> test=Ransac<float, Point2f, Homographie, 4>(data,thres,1000);
    Homographie H(data, rank, thres);

    //test.compute();

    float* param=H.param;
    cout<<param[0]<<"   "<<param[1]<<"   "<<param[2]<<endl;
    cout<<param[3]<<"   "<<param[4]<<"   "<<param[5]<<endl;
    cout<<param[6]<<"   "<<param[7]<<"   "<<param[8]<<endl;
    cout<<"erreur :  "<< H.get_error();


    return 0;
}
*/