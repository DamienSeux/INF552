#include <stdexcept>
#include <vector>
#include <stdlib.h>
#include <time.h>

using namespace std;

template <int n,typename k, typename F,int min>
/* n is the number of component for the data, k their type (float of int), n the min number of point for the model
 F is the class of the model
 F should have :
 - a min field : the minimum number of point to fit the model (static)
 - an init function fitting the model to the min first data points
 - a inlier function counting the number of inliers

 This is the RANSAC explain in class and used in open CV and not the usual version :
 the least square fit is only applied to the best model and there is no minimum number
 of inliers.
*/
class Ransac{
private:
    bool inliers[];
    F best; // the best model so far, or null if no model has been found
    int error; // score of the best model : the number of inliers
    k thres; // thresold to decide who is an insider
    int nstep; // how many random sample we try
    vector<vector<k>> data; // the data, is points of n components of type k.


public :
    Ransac(const vector<vector<k> >& data,k thres,int nstep):data(data),nstep(nstep),thres(thres){
        if(data.size()<min){
            throw invalid_argument("Pas assez de points");
        }
        inliers[data.size()];
    };

    void randomize(vector<vector<k> >& t){
        //sample min points at position 0..min-1 in t by swapping.
        srand((uint) time(NULL));
        if(min>t.size()){
            throw invalid_argument("Pas assez de points");
        }
        int ind;
        for(int i=0;i<min;i++){
            ind=rand() % (t.size()-i)+i;
            vector<k> temp=t[ind];
            t[ind]=t[i];
            t[i]=temp;
        }
    }

    void compute(){
        best=NULL;
        error=NULL;
        for(int i=0;i<nstep;i++){
            randomize(data);



        }






    }

    inline F get_best(){return best;}
    inline int get_error(){return error;}
    inline bool* get_inliers(){return inliers;}

};

int main(int argc, char *argv[]){

    return 0;
}