#include "LineDetector.hpp"
#include <stdio.h>
#include "TIMER.h"
namespace lut_detector{

bool LineDetector::detector(const Points& points, 
        const float& mindis, const float& maxdis, 
        const float& inter_ratio_, 
        std::vector<size_t>& rt_rect, size_t& w, size_t& h)
{
  TIMER timer("detector lines...");
  timer.start_timer();
// get the intersections in these points
  sinlines_.clear();
  sinlines_.reserve(points.size());
  inliner_points_ids_.clear();

// get lines by each points. min set 2PI, max set 0
  for(const auto& point : points){
      sinLine sl(point);
      sinlines_.push_back(sl);
  }// O(n)
  Threshold theta_threshold(-PI/2, 3*PI/2);
  Threshold r_threshold(mindis, maxdis);
  Points statistics_points;// in the (r,theta) coor
  // 在某一个参数范围内
  // 设置参数在某个参数范围内, theta 在[0,2pi], r[0,+∞]，delta_theta 2pi/30(角度上划分30份),
  // r的范围比较大，一般是按照像素大小来算为sqrt(w^2+h^2), 单位像素，可以隔10个像素一个块;
  float delta_theta = PI/1000.f;
  float delta_r = 5; //10px
  // 构造了2W的点需要去检索，花费的时间是多少
  // 创建统计块，并统计块内点的数量
  int rt_w = 0; int rt_h = 0;
  rt_w = std::ceil(theta_threshold.range()/delta_theta);  // 是
  rt_h = std::ceil(r_threshold.range()/delta_r);
  w = rt_w;
  h = rt_h;
  if(!rt_rect.empty())
    rt_rect.clear();
  rt_rect.resize(rt_w*rt_h);

  for (auto& line : sinlines_)
  {
      // one sinline can add same datas in the rect.
      float a = line.a_;
      float beta = line.beta_;
      int counter_rt_wi = 0;
      for(float theta=theta_threshold.min_; theta_threshold.isInter(theta); theta+= delta_theta){
          float r = line.getRRange(theta);
          int counter_rt_hi = std::ceil(r_threshold.max_-r)/delta_r;
          std::cout<<counter_rt_hi<<", "<<rt_w<<", "<<counter_rt_wi<<std::endl;
          rt_rect[counter_rt_hi*rt_w+counter_rt_wi] = 255;
          counter_rt_wi ++;
      }
    //   需要选择一些点绘制出来
  } 
  timer.end_timer();
  std::cout<<timer<<std::endl;
  return true;
}

// get the intersection of two sinline in the range og thetaThreshold.// now the theta_threshold is not used.
void LineDetector::sinLine::intersect(const sinLine& sl1, std::array<float, 3>& zerop){
    float gammas = a_*sin(beta_)-sl1.a_*sin(sl1.beta_);
    float gammac = a_*cos(beta_)-sl1.a_*cos(sl1.beta_);
    float gamma1 = std::sqrt(gammas*gammas+gammac*gammac);
    float sin_gamma = gammas/gamma1;
    float cos_gamma = gammac/gamma1;

    float gamma = asin(sin_gamma); // -PI/2 -- PI/2
    if(sin_gamma >=0){
        if(cos_gamma >= 0){
            // 一
            // keep it self 
            gamma = gamma;
        }
        else{
            // 二
            gamma = PI - gamma;
        }
    }else{
        if(cos_gamma < 0){
            // 三
            gamma = PI - gamma;
        }
        else{
            // 四
            // keep it self or change to 0-2PI
            gamma = 2*PI + gamma;
        }
    }
    // get intersections
    zerop[0] = 0-gamma;
    zerop[1] = PI - gamma;
    zerop[2] = 2*PI - gamma;
    
    std::vector<float> zero_result(3);
    if(std::fabs(zerop[0] + 2*PI) > 0.000001){
        // if the gamma nearly at the 2PI,will contained 3 zero result.
        zerop[0] = std::nan("");
    }
}

float LineDetector::sinLine::getRRange(const float& theta){
    // just has one result
    return a_*cos(theta-beta_);
}
}