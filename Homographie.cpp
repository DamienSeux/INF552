#include <stdexcept>
#include <vector>
#include <cmath>
#include <iostream>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

template<typename pt_type>

class Homographie {
public:
    pt_type param[3][3];
    bool inliers[];
    int error;

    Homographie(const vector<pair<pair<pt_type, pt_type>, pair<pt_type, pt_type>>> &data, const vector<int> &ranks, float thres) {
        pair<pair<pt_type, pt_type>, pair<pt_type, pt_type>> points1 = data[ranks[0]];
        pair<pair<pt_type, pt_type>, pair<pt_type, pt_type>> points2 = data[ranks[1]];
        pair<pair<pt_type, pt_type>, pair<pt_type, pt_type>> points3 = data[ranks[2]];
        pair<pair<pt_type, pt_type>, pair<pt_type, pt_type>> points4 = data[ranks[3]];
        bool inliers[data.size()];
        error = 0;

        if (points1 == points2 || points1 == points3 || points1 == points4 || points2 == points3 ||
            points2 == points4 || points3 == points4) {
            //est ce qu'on considere que si pointsx.first == pointsy.first alors pointsx == pointsy ?
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    param[i][j] == 0;
            error = data.size();
            for (int i = 0; i < data.size(); i++)
                inliers = false;
            return;
        }

        // création des matrices pour le calcul de l'homographie
        Mat H(3, 3, DataType<pt_type>::type);
        Mat A(3, 4, DataType<pt_type>::type);
        Mat B(3, 4, DataType<pt_type>::type);
        Mat S(3, 4, DataType<pt_type>::type);
        Mat U(3, 3, DataType<pt_type>::type);
        Mat tU(3, 3, DataType<pt_type>::type);
        Mat V(4, 4, DataType<pt_type>::type);
        Mat M(4, 3, DataType<pt_type>::type);

        for(int j = 0; j<4; j++){
            A.at<pt_type>(0, j) = data[ranks[j]].first.first;
            A.at<pt_type>(1, j) = data[ranks[j]].first.second;
            A.at<pt_type>(2, j) = 1;
            B.at<pt_type>(0, j) = data[ranks[j]].second.first;
            B.at<pt_type>(1, j) = data[ranks[j]].second.second;
            B.at<pt_type>(2, j) = 1;
        }

        //calcul de la pseudo inverse
        SVD::compute(A, S, U, V);

        for(int i = 0; i<3; i++)
            if(S.at<pt_type>(i, i) != 0)
                S.at<pt_type>(i, i) = 1/S.at<pt_type>(i, i);

        transpose(U, tU);
        M = V*S*tU;
        H = B*M;

        for(int i = 0; i<3; i++)
            for(int j = 0; j<3; j++)
                param[i][j] = H.at<pt_type>(i, j);

        bool flag;
        Mat p1(3, 1, DataType<pt_type>::type);
        Mat p2(3, 1, DataType<pt_type>::type);
        for(int i = 0; i<data.size(); i++){
            p1.at<pt_type>(0, 0) = data[i].first.first;
            p1.at<pt_type>(1, 0) = data[i].first.second;
            p1.at<pt_type>(2, 0) = 1;
            p2.at<pt_type>(0, 0) = data[i].second.first;
            p2.at<pt_type>(1, 0) = data[i].second.second;
            p2.at<pt_type>(2, 0) = 1;
            flag = (norm(H*p1 - p2)<thres);
            inliers[i] = flag;
            if(!flag)
                error++;
        }
        return;
    }

    Homographie(const Homographie<pt_type> &h, int n) {
        error = h.error;
        param = h.param;
        inliers = h.inliers;
    }

    Homographie() { };

    inline bool *get_inliers() {
        return inliers;
    }

    inline int get_error() const {
        return error;
    }


};