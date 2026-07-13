#ifndef SMOOTHER_HPP
#define SMOOTHER_HPP

#include <iostream>
#include "vector2d.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"

class Smoother
{
private:
    /// falloff rate for the voronoi field
    float alpha = 0.1;
    /// weight for the smoothness term
    float wSmoothness = 0.2;

    float map_origin_x;
    float map_origin_y;
    float map_height;
    float map_width;

public:
    Smoother(float origin_x, float origin_y, float height, float width) : map_origin_x(origin_x), map_origin_y(origin_y), map_height(height), map_width(width){};
    ~Smoother(){};
    bool isOnGrid(Vector2D vec);
    void smoothPath(nav_msgs::Path& path);
    Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);
    Vector2D smoothnessTerm(Vector2D xim1, Vector2D xi, Vector2D xip1);
};

bool Smoother::isOnGrid(Vector2D vec) {
    if (vec.getX() >= map_origin_x && vec.getX() < map_origin_x + map_width &&
        vec.getY() >= map_origin_y && vec.getY() < map_origin_y + map_height) {
      return true;
    }
    // return false;
    return true;
}

void Smoother::smoothPath(nav_msgs::Path& path){
    // current number of iterations of the gradient descent smoother
    int iterations = 0;
    // the maximum iterations for the gd smoother
    int maxIterations = 0;
    // the lenght of the path in number of nodes
    int pathLength = 0;

    pathLength = path.poses.size();
    nav_msgs::Path newPath = path;

    // descent along the gradient untill the maximum number of iterations has been reached
    float totalWeight = wSmoothness;

    while (iterations < maxIterations)
    {
        for (size_t i = 2; i < pathLength - 2; i++)
        {
            Vector2D xim2(newPath.poses[i - 2].pose.position.x, newPath.poses[i - 2].pose.position.y);
            Vector2D xim1(newPath.poses[i - 1].pose.position.x, newPath.poses[i - 1].pose.position.y);
            Vector2D xi(newPath.poses[i].pose.position.x, newPath.poses[i].pose.position.y);
            Vector2D xip1(newPath.poses[i + 1].pose.position.x, newPath.poses[i + 1].pose.position.y);
            Vector2D xip2(newPath.poses[i + 2].pose.position.x, newPath.poses[i + 2].pose.position.y);
            Vector2D correction;

            correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
            if (!isOnGrid(xi + correction)) { continue; }

            xi = xi + alpha * correction/totalWeight;
            newPath.poses[i].pose.position.x = xi.getX();
            newPath.poses[i].pose.position.y = xi.getY();
        }
        iterations++;
    }
    path = newPath;
}

Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
  return wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}

Vector2D Smoother::smoothnessTerm(Vector2D xim1, Vector2D xi, Vector2D xip1) {
  //使用了相邻的前面一个点、当前点、后面一个点
  return wSmoothness * (-4) * (xip1 - 2*xi + xim1);
}

#endif