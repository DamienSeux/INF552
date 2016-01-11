#include <stdexcept>
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

template<typename pt_type>

class Homographie {
public:
    pt_type param[3][3];
    bool inliers[];
    int error;

    Homographie(const vector<pair<pt_type, pt_type> > &data, const vector<int> &ranks, float thres) {
        pair<pt_type, pt_type> points1 = data[ranks[0]];
        pair<pt_type, pt_type> points2 = data[ranks[1]];
        pair<pt_type, pt_type> points3 = data[ranks[2]];
        pair<pt_type, pt_type> points4 = data[ranks[3]];
        bool inliers(data.size());

        if (points1 == points2 || points1 == points3 || points1 == points4 || points2 == points3 ||
            points2 == points4 || points3 == points4) {
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    param[i][j] == 0;
            error = data.size();
            for (int i = 0; i < data.size(); i++)
                inliers = false;
            return;
        }

        // construire les matrices avec les 3 points, trouver l'homographie, calculer l'erreur.
        //trouver un solveur linéaire
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