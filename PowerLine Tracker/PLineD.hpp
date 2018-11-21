#ifndef P_LINE_D_H_
#define P_LINE_D_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#define lineSeg std::vector< std::vector<cv::Point> >


namespace PLineD{
  void cutSeg(const lineSeg &src, lineSeg &dst, const size_t minLen=15 ,double angle = 67.5,const size_t step = 5);
  void contoursCanny(const cv::Mat &src,lineSeg &contours);
  void printContours(cv::Mat &dst, lineSeg &contours);
  void drawPoints(cv::Mat &dst,const std::vector<cv::Point> &pointList, const cv::Scalar color);
  void groupSeg(lineSeg src, lineSeg &dst,std::vector<cv::Point> &dst_angels,const size_t miP=300, const size_t miL = 100, double maA = 5, const size_t dS=20);
  void segCov(const lineSeg &cutS,lineSeg &reducedSeg, int ratio=500);
  void parallelSeg(const lineSeg &inPts, const std::vector<cv::Point> anglePts, lineSeg &paraSegments, double tAngle=5.0, size_t minPPL=2);
}
#endif
