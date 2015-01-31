#ifndef __mm_route__hh__
#define __mm_route__hh__

#include <string>
#include <vector>
#include "geometry.h"

class RTree;
class ShapefileGraph;
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
    
    bool ComputeCandidatePointSet(boost::shared_ptr<RTree> tree, double dist_thd);
    bool ComputeCandidateTrajectorySet(boost::shared_ptr<RTree> tree, boost::shared_ptr<ShapefileGraph> graph, double dist_thd);
    
    // 重采样
    void Resample(double res);
    
    inline bool isEmpty() { return route_.size() <= 0; }
    inline GPSTrajectory& getRoute() { return route_; }
    inline std::vector<CandidatePointSet>& getCandidateSet() { return candidate_sets_; }
    inline CandidatePoint getCandidatePoint(int gps_id, int candidate_id) { return candidate_sets_[gps_id][candidate_id]; }
    inline std::vector<CandidateTrajectory>& getCandidateTrajectories() { return candidate_trajectories_; }
    
    GPSPoint getRouteVertex(int index);
    
    void Reset();
    
private:
    // taxi route points
    //double sample_rate_;
    GPSTrajectory route_;
    std::vector<CandidatePointSet> candidate_sets_;
    std::vector<CandidateTrajectory> candidate_trajectories_;
};

#endif

