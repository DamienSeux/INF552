#include <stdexcept>
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

template <typename pt_type>
class Droite{
public:
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

