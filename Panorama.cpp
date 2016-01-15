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




int merge(Mat& I1,Mat& I2,Mat& output) {


        Mat img1,img2;
        cvtColor(I1,img1,CV_BGR2GRAY);
        cvtColor(I2,img2,CV_RGB2GRAY);
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

        /*drawKeypoints(img1, kpts1, img1_kp);
        drawKeypoints(img2, kpts2, img2_kp);
        imshow("AKAZE points", img1_kp);
        waitKey();
        imshow("AKAZE points", img2_kp);
        waitKey();*/

        // KNN matching

        BFMatcher matcher(NORM_HAMMING);
        vector<vector<DMatch> > nn_matches;
        matcher.knnMatch(desc1, desc2, nn_matches, 2);


        // On affiche le matching sans traitement
        drawMatches(img1, kpts1, img2, kpts2, nn_matches, img_corr);
       // imshow("Correspondance", img_corr);
       // waitKey();

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
                matches.push_back(DMatch((int) matched1.size() - 1,(int) matched2.size() - 1, 0));
            }
        }

        vector<pair<Point2f, Point2f>> data;
        for (int i = 0; i < matches.size(); i++) {
            data.push_back(pair<Point2f, Point2f>(matched1[i], matched2[i]));
        }

        /*Mat res;
        drawMatches(img1, matched1, img2, matched2, matches, res);
        imshow("Correspondance", res);
        waitKey();
         */


        // On trouve l'homographie
        Ransac<float, Point2f, Homographie, 4> findHomo = Ransac<float, Point2f, Homographie, 4>(data, thres, 500);
        findHomo.compute();

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                homo.at<float>(i, j) = findHomo.get_best().param[3 * i + j];


        // L'inverse est utile pour l'affichage final
        Mat homo_inv = homo.inv();


       /*drawMatches(img1, inliers1, img2, inliers2, good_matches, res);
       imshow("Correspondance", res);
       waitKey();*/

        // Affichage recollé


        warpPerspective(I2, res_final, homo_inv, cv::Size(I1.cols +I2.cols, I1.rows));
        Mat half(res_final, Rect(0, 0, I1.cols, I1.rows));

        Mat mask;
        cv::Scalar lowerb = cv::Scalar(0,0,0);
        cv::Scalar upperb = cv::Scalar(0,0,0);

        cv::inRange(I1, lowerb, upperb, mask);
        Mat mask2(mask.rows,mask.cols,CV_8U,1);
        mask2=mask2-mask;

        I1.copyTo(half,mask2);
        output = res_final;
    ~img1,img2,img1_kp,img2_kp;
    ~mask,mask2;

    return 0;
    }

    int main(void){
    Mat output;
    Mat I2 = imread("pano1/IMG_0045.JPG");

       for(int index = 44; index>38; index--) {
            cout<<index<<endl;
            std::stringstream sstm;
            sstm << "pano1/IMG_00" << index << ".JPG";
            string name = sstm.str();
            Mat I1 = imread(name);
            merge(I1,I2,output);
            I2=output;

    }
        Mat output2;
        Mat I3 = imread("pano1/IMG_0030.JPG");
        Mat dest1,dest2,dest3;
        flip(I3,dest1,1);

        for(int index = 31; index<39; index++) {
            cout<<index<<endl;
            std::stringstream sstm;
            sstm << "pano1/IMG_00" << index << ".JPG";
            string name = sstm.str();
            Mat I4 = imread(name);
            flip(I4,dest2,1);
            merge(dest2,dest1,output2);
            dest1=output2;

        }
        flip(output2,dest3,1);
        Mat outputfinal;
        merge(dest3,output,outputfinal);
        imshow("Grand pano",outputfinal);
        imwrite("Resultat.jpg",outputfinal);
        imwrite("Resultatgauche.jpg",dest3);
        imwrite("Resultatdroite.jpg",output);

    return 0;
    }


