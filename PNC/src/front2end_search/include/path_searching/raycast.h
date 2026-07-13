#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <cmath>
#include <limits>
#include <vector>

inline double signum(double x)
{
  return x == 0.0 ? 0.0 : (x < 0.0 ? -1.0 : 1.0);
}

inline double mod(double value, double modulus)
{
  return std::fmod(std::fmod(value, modulus) + modulus, modulus);
}

inline double intbound(double s, double ds)
{
  if (ds == 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  if (ds < 0.0) {
    return intbound(-s, -ds);
  }
  s = mod(s, 1.0);
  return (1.0 - s) / ds;
}

// void Raycast(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const Eigen::Vector2d& min,
//              const Eigen::Vector2d& max, int& output_points_cnt, Eigen::Vector2d* output);

// void Raycast(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const Eigen::Vector2d& min,
//              const Eigen::Vector2d& max, std::vector<Eigen::Vector2d>* output);

class RayCaster
{
private:
  /* data */
  Eigen::Vector2d start_;
  Eigen::Vector2d end_;
  Eigen::Vector2d direction_;
  Eigen::Vector2d min_;
  Eigen::Vector2d max_;
  int x_;
  int y_;
  // int z_;
  int endX_;
  int endY_;
  // int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  // double dz_;
  int stepX_;
  int stepY_;
  // int stepZ_;
  double tMaxX_;
  double tMaxY_;
  // double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  // double tDeltaZ_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */)
  {
  }
  ~RayCaster()
  {
  }

  bool setInput(const Eigen::Vector2d& start, const Eigen::Vector2d& end/* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */)
  {
    start_ = start;
    end_ = end;
    direction_ = end_ - start_;

    x_ = static_cast<int>(std::floor(start_.x()));
    y_ = static_cast<int>(std::floor(start_.y()));
    endX_ = static_cast<int>(std::floor(end_.x()));
    endY_ = static_cast<int>(std::floor(end_.y()));

    dx_ = direction_.x();
    dy_ = direction_.y();
    stepX_ = static_cast<int>(signum(dx_));
    stepY_ = static_cast<int>(signum(dy_));

    tMaxX_ = intbound(start_.x(), dx_);
    tMaxY_ = intbound(start_.y(), dy_);
    tDeltaX_ = stepX_ == 0 ? std::numeric_limits<double>::infinity() : static_cast<double>(stepX_) / dx_;
    tDeltaY_ = stepY_ == 0 ? std::numeric_limits<double>::infinity() : static_cast<double>(stepY_) / dy_;

    dist_ = 0.0;
    maxDist_ = direction_.norm();
    step_num_ = 0;
    return maxDist_ > 1e-9;
  }

  bool step(Eigen::Vector2d& ray_pt)
  {
    if (step_num_ == 0) {
      ray_pt = Eigen::Vector2d(x_, y_);
      step_num_++;
      return true;
    }

    if (x_ == endX_ && y_ == endY_) {
      return false;
    }

    if (tMaxX_ < tMaxY_) {
      x_ += stepX_;
      dist_ = tMaxX_;
      tMaxX_ += tDeltaX_;
    } else {
      y_ += stepY_;
      dist_ = tMaxY_;
      tMaxY_ += tDeltaY_;
    }

    if (dist_ > 1.0) {
      return false;
    }

    ray_pt = Eigen::Vector2d(x_, y_);
    step_num_++;
    return true;
  }
};

#endif  // RAYCAST_H_
