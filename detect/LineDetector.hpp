#pragma once

#include <vector>
#include <math.h>
#include <array>
#include <iostream>
#define PI 3.14159262

namespace lut_detector{
    typedef size_t line_t;
    typedef size_t point_t;
    struct Point{
      float x_ = std::nan("");
      float y_ = std::nan("");
      Point() = default;
      Point(const float& x, const float& y):x_(x), y_(y){}
    };
    
    struct Threshold{
      float min_ = std::nan("");
      float max_ = std::nan("");
      Threshold():min_(0), max_(2*PI){}
      Threshold(const float& i, const float& a):min_(i), max_(a){}

      bool isInter(const float& value){
        if(min_ > max_){
            printf("threshold range error. %f < %f.\n", min_, max_);
            exit(-1);
        }
        if(value >= min_ && value <= max_) return true; else return false;
      }
      float range(){return max_-min_;}
    };
    typedef std::vector<Point> Points;
    
    class LineDetector{
        // 应该还有一些容错的阈值
    public:
        std::vector<point_t> inliner_points_ids_;

        float detect_accuracy_ = 0.1;
        // 1个点存在一条曲线，1千个点存在1k条曲线，如何快速的搜索。
        // r = a sin(beta+theta);
        struct sinLine{
          // read the README.md,公式
          float a_;
          float beta_;
          Points points_;
          sinLine(const Point& point){
            a_ = std::sqrt(point.x_*point.x_+point.y_*point.y_);
            beta_ = acos(point.x_/a_); // (0,pi/2)
            // x > 0, y > 0，因此任何一条线
            points_.push_back(point);
          }
          // if reconstruction is true, the x,y means r and theta
          sinLine(const float& x,const float& y, const bool& is_reconstruction = true){
            if(is_reconstruction){
                a_ = std::sqrt(x*x+y*y);
                beta_ = acos(x/a_);
                // x>0, y>0因此sin/cos(beta)>0,因此这里是没有象限判断的。
            }else{a_ = x; beta_ = y;}
          }
          void intersect(const sinLine& sl1, std::array<float, 3>& zerop);
          float getRRange(const float& theta);
        };
        typedef std::vector<sinLine> SinLines;
        SinLines sinlines_;

        LineDetector() = default;

        bool detector(const Points& points, 
          const float& min_dis, const float& max_dis, 
          const float& inter_ratio, 
          std::vector<size_t>& rt_wh, size_t& w, size_t& h);

    };
}