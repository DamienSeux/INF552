

/* n is the number of component for the data, pt_type their type (float of int), n the min number of point for the model
 F is the class of the model, whose parameters are computed when constructor is called.

 This is the RANSAC explain in class and used in open CV and not the usual version :
 the least square fit is only applied to the best model and there is no minimum number
 of inliers.
*/
template <typename pt_type, typename dt_type, typename F,int min>
class RANSAC{
private:
    bool inliers[];
    F best; // the best model so far, or null if no model has been found
    int error; // score of the best model : the number of inliers
    pt_type thres; // thresold to decide who is an insider
    int nstep; // how many random sample we try
    vector<pair<dt_type,dt_type > > data; // the data, is points of n components of type pt_type.
    vector<int> ranks;

public :
    RANSAC(const vector<pair<dt_type,dt_type> >& data,pt_type thres,int nstep):data(data),nstep(nstep),thres(thres){
        if(data.size()<min){
            throw invalid_argument("Pas assez de points");
        }
        inliers[data.size()];
        vector<int> ranks;
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


int main(int argc, char *argv[]){

    vector<pair<float,float> > data;
    srand((uint) time(NULL));
    for(int i=0;i<200;i++){
        data.push_back(pair<float,float>(i,5*i+12+(rand() % (100)-50)/(float) 100));
    }
    RANSAC<float,float,Droite,2> test(data,2,100);

    test.compute();

    float* param=test.get_best().param;
    cout<<param[0]<<","<<param[1]<<param[2]<<endl;
    cout<<test.get_best().get_error();





    return 0;
}