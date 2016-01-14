#include <stdexcept>
#include <vector>
#include <cmath>
#include <iostream>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;



class Homographie {
public:
    float param[9];
    bool inliers[];
    int error;

    Homographie(const vector<pair<Point2f, Point2f>> &data, const vector<int> &ranks, float thres) {

        vector<pair<Point2f, Point2f>> points;
        points.push_back(data[ranks[0]]);
        points.push_back(data[ranks[1]]);
        points.push_back(data[ranks[2]]);
        points.push_back(data[ranks[3]]);

        bool inliers[data.size()];
        error = 0;

        // création des matrices pour le calcul de l'homographie
        Mat H(3, 3, CV_32F);
        Mat HCol(9, 1, CV_32F);
        Mat A(8, 9, CV_32F);
        Mat S(8, 1, CV_32F);
        Mat U(8, 8, CV_32F);
        Mat V(9, 9, CV_32F);
        Mat X(9, 1, CV_32F);

        //on remplit les matrices
        for (int i = 0; i < 4; i++) {
            A.at<float>(2 * i, 0) = -points[i].first.x;
            A.at<float>(2 * i, 1) = -points[i].first.y;
            A.at<float>(2 * i, 2) = -1.0;
            A.at<float>(2 * i, 3) = 0.0;
            A.at<float>(2 * i, 4) = 0.0;
            A.at<float>(2 * i, 5) = 0.0;
            A.at<float>(2 * i, 6) = points[i].second.x * points[i].first.x;
            A.at<float>(2 * i, 7) = points[i].second.x * points[i].first.y;
            A.at<float>(2 * i, 8) = points[i].second.x;

            A.at<float>(2 * i + 1, 0) = 0.0;
            A.at<float>(2 * i + 1, 1) = 0.0;
            A.at<float>(2 * i + 1, 2) = 0.0;
            A.at<float>(2 * i + 1, 3) = -points[i].first.x;
            A.at<float>(2 * i + 1, 4) = -points[i].first.y;
            A.at<float>(2 * i + 1, 5) = -1.0;
            A.at<float>(2 * i + 1, 6) = points[i].second.y * points[i].first.x;
            A.at<float>(2 * i + 1, 7) = points[i].second.y * points[i].first.y;
            A.at<float>(2 * i + 1, 8) = points[i].second.y;
        }

        //calcul de la pseudo inverse
        SVD::compute(A, S, U, V, SVD::FULL_UV);

        //on recupere la valeur singuliere minimale
        int index = 8;

        for (int i = 0; i < 9; i++)
            X.at<float>(0, i) = 0;

        X.at<float>(0, index) = 1;

        for (int i = 0; i < 9; i++)
            X.at<float>(0, i) = 0;

        X.at<float>(0, index) = 1;

        HCol = V.t() * X;


        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++) {
                H.at<float>(i, j) = HCol.at<float>(0,i*3 + j);
            }
        }

        for(int i = 0; i<3; i++)
            for(int j = 0; j<3; j++)
                param[3*i+j] = H.at<float>(i, j);

        bool flag;
        Mat p1(3, 1, DataType<float>::type);
        Mat p2(3, 1, DataType<float>::type);
        Mat p3(3, 1, DataType<float>::type);
        for(int i = 0; i<data.size(); i++){
            p1.at<float>(0, 0) = data[i].first.x;
            p1.at<float>(1, 0) = data[i].first.y;
            p1.at<float>(2, 0) = 1;
            p2.at<float>(0, 0) = data[i].second.x;
            p2.at<float>(1, 0) = data[i].second.y;
            p2.at<float>(2, 0) = 1;
            p3= H*p1;
            float alpha = p3.at<float>(2, 0);
            if(alpha!=0)
                flag = (norm(H*p1*(1/alpha) - p2)<thres);
            else flag = false;
            inliers[i] = flag;
            if(!flag)
                error++;
        }

        return;
    }

    Homographie(const Homographie &h, int n) {
        error = h.error;

        for(int i = 0; i<9; i++)
            param[i] = h.param[i];
        for(int i = 0; i < n; i++)
            inliers[i] = h.inliers[i];
    }
    Homographie(const Homographie &h) {
        error = h.error;

        for(int i = 0; i<9; i++)
            param[i] = h.param[i];
    }

    Homographie() { };

    inline bool *get_inliers() {
        return inliers;
    }

    inline int get_error() const {
        return error;
    }

};