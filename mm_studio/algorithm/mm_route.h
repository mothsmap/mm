#ifndef __mm_route__hh__
#define __mm_route__hh__

#include <string>
#include <vector>
#include "geometry.h"

class RTree;
// GPS points
class Route {
public:
    Route(void);
     ~Route(void);

    // 从shapefile中加载
    bool Load(std::string);
    
    // 插入顶点
    void InsertNode(double x, double y, int gps_time, int head, int speed);
    
    void InsertCandidatePoint(CandidatePoint candidate_point);
    
    void ComputeCandidateTrajectorySet(boost::shared_ptr<RTree> tree);
    
    // 重采样
    void Resample(double res);
    
    inline bool isEmpty() { return route_.size() <= 0; }
    inline GPSTrajectory& getRoute() { return route_; }
    inline std::vector<CandidatePointSet>& getCandidateSet() { return candidate_sets_; }
    inline std::vector<CandidateTrajectory>& getCandidateTrajectories() { return candidate_trajectories_; }
    
    GPSPoint getRouteVertex(int index);
    
    void Reset();
    
private:
    // taxi route points
    GPSTrajectory route_;
    std::vector<CandidatePointSet> candidate_sets_;
    std::vector<CandidateTrajectory> candidate_trajectories_;
};

#endif
