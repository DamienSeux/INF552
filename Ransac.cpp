#include <stdexcept>
#include <vector>
#include <cmath>

using namespace std;

template <typename pt_type, typename F,int min>
/* n is the number of component for the data, pt_type their type (float of int), n the min number of point for the model
 F is the class of the model
 F should have :
 - a min field : the minimum number of point to fit the model (static)
 - an init function fitting the model to the min first data points
 - a inlier function counting the number of inliers

 This is the RANSAC explain in class and used in open CV and not the usual version :
 the least square fit is only applied to the best model and there is no minimum number
 of inliers.
*/
class RANSAC{
private:
    bool inliers[];
    F best; // the best model so far, or null if no model has been found
    int error; // score of the best model : the number of inliers
    pt_type thres; // thresold to decide who is an insider
    int nstep; // how many random sample we try
    vector<pair<vector<pt_type>,vector<pt_type> > > data; // the data, is points of n components of type pt_type.
    vector<int> ranks;

public :
    RANSAC(const vector<pair<vector<pt_type>,vector<pt_type> > >& data,pt_type thres,int nstep):data(data),nstep(nstep),thres(thres){
        if(data.size()<min){
            throw invalid_argument("Pas assez de points");
        }
        inliers[data.size()];
        vector<int> ranks;
        for(int i=0;i<data.size();i++){
            ranks.push_back(i);
        }
    };

    void copy(bool src[],bool dst[],int size){
        for(int i=0;i<size;i++){
            dst[i]=src[i];
        }
    }

    void randomize(vector<int>& t){
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

    void compute(){
        best=NULL;
        error=NULL;
        for(int i=0;i<nstep;i++){
            randomize(ranks);
            F model(data,ranks);
            if(model.get_error()<error){
                best=model;
                error=model.get_error();
                copy(model.get_inliers(),inliers,data.size());
            }
            delete model;
        }


    }

    inline F get_best(){return best;}
    inline int get_error(){return error;}
    inline bool* get_inliers(){return inliers;}

};
template <typename pt_type>
struct Droite{
    pt_type param[3];
    bool inliers[];
    int error;

    void normalize(pt_type param[3] ){
        float norm=sqrt(param[0]*param[0]+param[1]*param[1]+param[2]*param[2]);
        if(norm>0){
            param[0]=param[0]/norm;
            param[1]=param[1]/norm;
            param[2]=param[2]/norm;
        }
    }

    Droite(const vector<pair<pt_type,pt_type> >& data,const vector<int>& ranks,pt_type thres):inliers(new bool[data.size()]){
        pair<pt_type,pt_type> point1=data[ranks[0]];
        pair<pt_type,pt_type> point2=data[ranks[1]];
        if(point1==point2){
            param[0]=0;
            param[1]=0;
            param[2]=0;
            error=data.size();
            for(int i=0;i<data.size();i++){
                inliers[i]=false;
            }
            return;
        }

        if(point1.first==point2.first){
            param[0]=0;
            param[1]=1;
            param[2]=-1*point1.first;
            error=0;
            bool flag;
            for(int i=0;i<data.size();i++){
                flag=(data[i].first+param[2]<thres);
                inliers[i]=flag;
                if(!flag)
                    error++;
            }
            return;
        }else{
            param[0]=(point2.second-point1.second);
            param[1]=(point1.first-point2.first);
            param[2]=-1*(param[0]*point1.first+param[1]*point1.second);
            normalize(param);
            pair<pt_type,pt_type> origin=pair<pt_type,pt_type>(0,-param[2]/param[1]);
            bool flag;
            error=0;
            for(int i=0;i<data.size();i++){
                flag=(data[i].first-origin.first)*param[0]+(data[i].second-origin.second)*param[1]<thres;
                inliers[i]=flag;
                if(!flag)
                    error++;
            }
          return;
        }

    }
    Droite(const Droite<pt_type>& d):inliers(d.inliers),error(d.error){
        pt_type param[2];
        param[0]=d.param[0];
        param[1]=d.param[1];
    }
    inline bool* get_inliers() const{
        return inliers;
    }
    inline int get_error() const{
        return error;
    }


};

int main(int argc, char *argv[]){

    return 0;
}