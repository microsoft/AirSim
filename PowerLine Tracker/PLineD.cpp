#include "PLineD.hpp"
#include <math.h>

cv::RNG rng(cv::getTickCount());
namespace {
  struct lineSeg_less_than_key{
      inline bool operator() (const std::vector<cv::Point>& struct1, const std::vector<cv::Point>& struct2)
      {
          return (struct1.size() > struct2.size());
      }
  };
  double diffAngle(std::vector<cv::Point> vec1, std::vector<cv::Point> vec2){
      cv::Point V1 = vec1.front()-vec1.back();
      cv::Point V2 = vec2.front()-vec2.back();
      double A= acos(V1.dot(V2)  /  (sqrt(   V1.dot(V1)   )*sqrt(  V2.dot(V2)  )));
      if(A > M_PI/2) A = M_PI-A;
      return A;
  }
  cv::Mat calcVecCovar(const std::vector<cv::Point> &inRow){
      double varXY, varXX, varYY =0;
      cv::Mat coVarMatx = cv::Mat(2,2,CV_64F);
      for (size_t i = 0; i < inRow.size(); i++) {
          varXX += (inRow[i].x * inRow[i].x);
          varXY += (inRow[i].x * inRow[i].y);
          varYY += (inRow[i].y * inRow[i].y);
      }
      varXX /= inRow.size();
      varXY /= inRow.size();
      varYY /= inRow.size();
      coVarMatx.at<double>(0,0) = varXX;
      coVarMatx.at<double>(0,1) = varXY;
      coVarMatx.at<double>(1,0) = varXY;
      coVarMatx.at<double>(1,1) = varYY;
      return coVarMatx;
  }
  double calcDiffAngle(cv::Point V1, cv::Point V2){
    double A= acos(V1.dot(V2)  /  (sqrt(   V1.dot(V1)   )*sqrt(  V2.dot(V2)  )));
    if(A > M_PI/2) A = M_PI-A;
    return A;
  }
}

namespace PLineD{
  void cutSeg(const lineSeg &src,lineSeg &dst,const size_t minLen ,double angle,const size_t step){
      double A;
      cv::Point V1,V2;
      size_t last,i,z;

      angle *= (M_PI/180);
      for(i = 0; i< src.size(); i++ ){
          if(src[i].size()>minLen){
              if(dst.size() >0 && dst.back().size() == 1) dst.pop_back();
              dst.push_back(std::vector<cv::Point>());
              last = 0;
              for(z = 0; z < src[i].size(); z++ ){
                  dst.back().push_back(src[i][z]);
                  if(z-step>=last && z+step <src[i].size()){
                      V1 = src[i][z] -src[i][z-step];
                      V2 = src[i][z+step] -src[i][z];
                      A = acos(V1.dot(V2)  /  (sqrt(   V1.dot(V1)   )*sqrt(  V2.dot(V2)  )));
                      if(A>angle){
                          last = z;
                          if(dst.back().size() == 1) dst.pop_back();
                          dst.push_back(std::vector<cv::Point>());

                      }
                  }
              }
          }
      }
  }
  void contoursCanny(const cv::Mat &src,lineSeg &contours){
      std::vector<cv::Vec4i> hierarchy;
      cv::Mat img;
      cv::blur(src,img,cv::Size(3,3));
      Canny(img, img, 36, 150, 3);
      findContours( img, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
  }
  void printContours(cv::Mat &dst,lineSeg &contours){
      for( size_t i = 0; i< contours.size(); i++ ){
          cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
          for( size_t z = 0; z < contours[i].size(); z++ ){
              dst.at<cv::Vec3b>(contours[i][z]) = {uchar(color[0]), uchar(color[1]), uchar(color[2])};
          }
      }
  }
  void drawPoints(cv::Mat &dst,const std::vector<cv::Point> &pointList, const cv::Scalar color){
      for(size_t i = 0; i < pointList.size(); i++){
          dst.at<cv::Vec3b>(pointList[i]) = {uchar(color[0]), uchar(color[1]), uchar(color[2])};
      }

  }
  void groupSeg(lineSeg src, lineSeg &dst,std::vector<cv::Point> &dst_angels,const size_t miP, const size_t miL, double maA, const size_t dS){
      maA*=(M_PI/180);
      std::sort(src.begin(), src.end(), lineSeg_less_than_key());
      dst.push_back(src.front());
      dst_angels.push_back(src.front().front()-src.front().back());
      for(size_t i = 0; i < src.size(); i++){
          if(src[i].size()>miL){
              for(size_t z = i+1; z < src.size(); z++){
                  if(diffAngle(src[i],src[z]) < maA){
                      cv::Point V1 = src[i].back()-src[i].front();
                      cv::Point V2 = src[z][src[z].size()/2] - src[i].back();
                      double dist = std::abs(V1.cross(V2))/sqrt(V1.dot(V1));
                      if(dist < dS){
                          dst.back().insert(dst.back().end(),src[z].begin(),src[z].end());
                          src.erase(src.begin()+long(z));
                          z--;
                      }
                  }
              }
          }else{
              break;
          }
          if(src.size()>i+1){
              if(dst.back().size()>miP){
                  dst.push_back(src[i+1]);
                  dst_angels.push_back(src[i+1].front()-src[i+1].back());
              }else{
                  dst.back().clear();
                  dst_angels.back() =src[i+1].front()-src[i+1].back();
              }
          }

      }
  }
  void segCov(const lineSeg &cutS,lineSeg &reducedSeg, int ratio){
      for (size_t i = 0; i < cutS.size(); i++) {
          cv::Mat COV, eigenVals;
          if (cutS[i].size() > 1){
              COV = calcVecCovar(cutS[i]);
              cv::eigen(COV, eigenVals);
              double lambda1 = eigenVals.at<double>(0,0);
              double lambda2 = eigenVals.at<double>(0,1);

              if ((lambda1/lambda2) < ratio ){
                  reducedSeg.push_back(cutS[i]);
              }
          }
          else{
          }
      }
  }
  void parallelSeg(const lineSeg &inPts, const std::vector<cv::Point> anglePts, lineSeg &paraSegments, double tAngle, size_t minPPL){
      lineSeg tmpSegments;
      for (size_t i = 0; i < inPts.size(); i++) {
        // cout << i << ": " << endl << endl;

        for (size_t j = 0; j < inPts.size(); j++) {
          float diffAngle = calcDiffAngle(anglePts[i], anglePts[j]) * (180/M_PI);
          if (i != j){
            // cout << "i: " << i << " j: " << j << " AngleDif: " << diffAngle;
          if (diffAngle < tAngle){
              // cout << " pushed";
              tmpSegments.push_back(inPts[i]);
           }
           // cout << endl;
         }
         }
      }
    }
}
