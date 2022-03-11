#ifndef DBSCAN_H
#define DBSCAN_H

#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <vector>
#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;

class DBSCAN {
 public:
  DBSCAN(unsigned int minPts, float eps, vector<Point> points) {
    m_minPoints = minPts;
    m_epsilon = eps;
    m_points = pc;
    m_pointSize = points.size();
  }
  ~DBSCAN() {}

  int run();
  vector<int> calculateCluster(Point point);
  int expandCluster(Point point, int clusterID);
  inline double calculateDistance(const Point& pointCore,
                                  const Point& pointTarget);

  int getTotalPointSize() { return m_pointSize; }
  int getMinimumClusterSize() { return m_minPoints; }
  int getEpsilonSize() { return m_epsilon; }

 public:
  sensor_msgs::PointCloud pc;

 private:
  unsigned int m_pointSize;
  unsigned int m_minPoints;
  float m_epsilon;
};