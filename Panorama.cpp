//
// Created by quentin on 14/01/16.
//

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include "Ransac.cpp"
#include <sstream>

// Enable ORB instead of AKAZE
//#define ORB

using namespace std;
using namespace cv;

const float thres = 1.5f; // Distance threshold to identify inliers
const float nn_match_ratio = 0.6f;   // Nearest neighbor matching ratio

int main(void)
{
    Mat I1 = imread("pano1/IMG_0027.JPG");
    Mat img1;
    for(int index = 28; index<29; index++) {
        std::stringstream sstm;
        sstm << "pano1/IMG_00" << index << ".JPG";
        string name = sstm.str();
        Mat I2 = imread(name);
        Mat img2 = imread(name, IMREAD_GRAYSCALE);
        cvtColor(I1,img1,CV_RGB2GRAY);
        Mat img1_kp, img2_kp, img_corr, res_final;
        Mat homo(3, 3, DataType<float>::type);

        vector<KeyPoint> kpts1, kpts2;
        Mat desc1, desc2;

#ifndef ORB
        // AKAZE
        Ptr<AKAZE> akaze = AKAZE::create();
        akaze->detectAndCompute(img1, noArray(), kpts1, desc1);
        akaze->detectAndCompute(img2, noArray(), kpts2, desc2);
#else
        // ORB
        Ptr<ORB> orb = ORB::create();
        orb->detectAndCompute(img1, noArray(), kpts1, desc1);
        orb->detectAndCompute(img2, noArray(), kpts2, desc2);
#endif

        drawKeypoints(img1, kpts1, img1_kp);
        drawKeypoints(img2, kpts2, img2_kp);
        imshow("AKAZE points", img1_kp);
        waitKey();
        imshow("AKAZE points", img2_kp);
        waitKey();

        // KNN matching

        BFMatcher matcher(NORM_HAMMING);
        vector<vector<DMatch> > nn_matches;
        matcher.knnMatch(desc1, desc2, nn_matches, 2);


        // On affiche le matching sans traitement
        drawMatches(img1, kpts1, img2, kpts2, nn_matches, img_corr);
        imshow("Correspondance", img_corr);
        waitKey();

        // Premier traitement

        vector<Point2f> matched1, matched2, inliers1, inliers2;
        vector<DMatch> good_matches, matches;
        for (size_t i = 0; i < nn_matches.size(); i++) {
            DMatch first = nn_matches[i][0];
            float dist1 = nn_matches[i][0].distance;
            float dist2 = nn_matches[i][1].distance;

            if (dist1 < nn_match_ratio * dist2) {
                matched1.push_back(kpts1[first.queryIdx].pt);
                matched2.push_back(kpts2[first.trainIdx].pt);
                matches.push_back(DMatch(matched1.size() - 1, matched2.size() - 1, 0));
            }
        }

        vector<pair<Point2f, Point2f>> data;
        for (int i = 0; i < matches.size(); i++) {
            data.push_back(pair<Point2f, Point2f>(matched1[i], matched2[i]));
        }

        Mat res;
       // drawMatches(img1, matched1, img2, matched2, matches, res);
       // imshow("Correspondance", res);
        waitKey();


        // On trouve l'homographie
        Ransac<float, Point2f, Homographie, 4> findHomo = Ransac<float, Point2f, Homographie, 4>(data, thres, 100);
        findHomo.compute();
        cout<<findHomo.get_error()<<endl;
        cout<<data.size()<<endl;

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                homo.at<float>(i, j) = findHomo.get_best().param[3 * i + j];


        cout << homo << endl<<endl;
        // L'inverse est utile pour l'affichage final
        Mat homo_inv = homo.inv();

        // On trouve les inliers

        for (unsigned i = 0; i < matched1.size(); i++) {
            Mat col = Mat::ones(3, 1, CV_32F);
            col.at<double>(0) = matched1[i].x;
            col.at<double>(1) = matched1[i].y;

            col = homo * col;
            col /= col.at<double>(2);
            double dist = sqrt(pow(col.at<double>(0) - matched2[i].x, 2) +
                               pow(col.at<double>(1) - matched2[i].y, 2));

            if (dist < thres) {
                int new_i = static_cast<int>(inliers1.size());
                inliers1.push_back(matched1[i]);
                inliers2.push_back(matched2[i]);
                good_matches.push_back(DMatch(new_i, new_i, 0));
            }
        }


       // drawMatches(img1, inliers1, img2, inliers2, good_matches, res);
       // imshow("Correspondance", res);
        waitKey();

        // Affichage recollé

        warpPerspective(I2, res_final, homo_inv, cv::Size(I1.cols * 2, I1.rows));
        Mat half(res_final, Rect(0, 0, I1.cols, I1.rows));
        I1.copyTo(half);
        imshow("Result", res_final);
        waitKey();
        I1 = res_final;
    }

    return 0;
}

