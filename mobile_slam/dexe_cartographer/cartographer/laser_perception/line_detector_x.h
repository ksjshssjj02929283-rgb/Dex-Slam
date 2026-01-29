#ifndef CARTOGRAPHER__LASER_PERCEPTION__LINE_DETECTOR_X_H_
#define CARTOGRAPHER__LASER_PERCEPTION__LINE_DETECTOR_X_H_

#include <iostream>
#include <list>
#include <utility>
#include <vector>
#include <stdio.h>
#include <math.h>

#include "cartographer/laser_perception/point_set.h"
#include "cartographer/laser_perception/segment.h"
#include "cartographer/laser_perception/config.h"
#include "cartographer/laser_perception/visualization.h"

namespace cartographer
{
namespace laser_perception
{

#define threshold_a 6     //聚类倍数
#define distance_point(a1,a2,b1,b2) sqrt((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2))


// detector所需要的数据结构
typedef struct _POINT
{
  double x;
  double y;
}POINT;

typedef struct _CSData
{
  std::vector<unsigned int> index;//索引值
  std::vector<double> bearings;//角度
  std::vector<double> cos_value;//余弦
  std::vector<double> sin_value;//正弦
}CSdata;

typedef struct _RangeData
{
  std::vector<double> ranges;//role数值
  std::vector<double> xs;//x坐标
  std::vector<double> ys;//y坐标
}Rangedata;

//参数，从launch文件中读入
typedef struct _Params
{
  double angle_increment;//角度增量
  double angle_start;//初始角度
  double least_thresh;//正交拟合阈值
  double min_line_length;//拟合线段最短距离
  double predict_distance;//真实点与与预测点之间的距离阈值
  unsigned int min_line_points;//一条线段包含的激光点个数
  unsigned int seed_line_points;//种子线段包含的激光点个数
}Params;


typedef struct _word_params
{
  double _role;
  double _theta_one;
  double _theta_two;
}word_params;

typedef struct _signal_params
{
  double distance_signal;
}signal_params;

//直线段信息结构体
typedef struct _line
{
  double a;//直线参数
  double b;
  double c;
  int left;//直线范围
  int right;
  POINT p1;
  POINT p2;
  bool inte[2];
}line;

//直线方程式结构体
typedef struct _least
{
  double a;
  double b;
  double c;
}least;

//}

typedef struct _point
{
  double role;
    double theta;
  double m_x;
    double m_y;
    double distance;
  double m_gradient;
  bool flag;
}PoinT;
  

typedef struct _generate_line
{
  //first point
  double x1;
  double y1;
  //end point
  double x2;
  double y2;
}gline;

typedef struct _signal
{
  double _angle1_radian;
  double _angle2_radian;
  double _angle1_degree;
  double _angle2_degree;
  double _role;
}Signal;

typedef struct _feature_point
{
  POINT _point;
  double _angle;
}featurepoint;

typedef struct _keyword
{
  int _index_role;
  int _index_theta_one;
  int _index_theta_two;
  std::vector<int> _frame_index;
}keyword;


class LineDetectorX
{
public:
  LineDetectorX();
  
  ~LineDetectorX();

  std::list<Segment> detectLines(const std::list<Point> input_points);

  //设置参数
  void set_angle_increment(double);
  void set_angle_start(double);
  void set_least_threshold(double);
  void set_min_line_length(double);
  void set_predict_distance(double);
  void set_min_line_points(unsigned int);
  void set_seed_line_points(unsigned int);

private:
  //设置bearing，一次设置就行
  void setCosSinData(const std::vector<double>&, const std::vector<double>&,const std::vector<double>&, const std::vector<unsigned int>&);
  //返回直线分割结果
  void extractLines(std::vector<line>&,std::vector<gline>&);
  void computeBearing(const std::list<Point> input_points);
  void setRangeData(const std::list<Point> input_points);

  //检测种子点，障碍物聚类的子函数
  //bool detectseed(PoinT );
  //障碍物检测，区域生长
  //int regiongrow(int);
  //通过激光数据的首末索引值进行直线方程的求解
  least leastsquare(int,int,int);
  //检测种子直线
  bool detectline(const int,const int);
  //通过种子直线，复原出整条直线，并进行最后的求定
  int detectfulline(const int);
  //整理整条直线
  void cleanline();
  //删除小于长度阈值的线段
  bool delete_short_line(const int,const int);
  //
  void generate(std::vector<gline>& temp_line2);
private:

  CSdata cs_data_;
  Rangedata range_data_;
  Params params_;
  unsigned int point_num_;
  //线段结构体信息
  std::vector<line> m_line;
  //直线拟合中间传递变量，已设为全局变量
  least m_least;
  //拟合中间变量
  double mid1;
  double mid2;
  double mid3;
  double mid4;
  double mid5;

  bool is_cs_data_set_ = false;
};

}  // namespace laser_perception
}  // namespace cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__LINE_DETECTOR_X_HPP_
