#ifndef __mm__density__hh__
#define __mm__density__hh__

#include <boost/shared_ptr.hpp>
#include <vector>
#include "boost_geometry.h"

class RTree;
class Route;
class ShapefileGraph;

class MMDensity {
public:
    MMDensity(boost::shared_ptr<RTree> rtree,
              boost::shared_ptr<Route> route,
              boost::shared_ptr<ShapefileGraph> shapefile_graph);
    
    ~MMDensity();
    
    std::vector<int>& Match();

    // 匹配一个GPS点到一条边
    // point_id: GPS点的id
    // edge: 匹配的边
    // score: 得分
    // inside: 该点匹配到边的内部还是端点
    // start_x, start_y: 即是输入也是输出，输入表示当前匹配边的起点，输出表示下一条边的起点
    void MatchPoint2Edge(int point_id, int edge, int pre_edge, double& score, bool& inside, int& start);

    // 匹配一个GPS点到候选边集中的一条边
    // point_id: GPS点的id
    // edges: 边集
    // score: 得分
    // inside: 该点匹配到边的内部还是端点
    // edge_id: 匹配到的边的id
    // start_x, start_y: 即是输入也是输出，输入表示当前匹配边的起点，输出表示下一条边的起点
    void MatchPoint2EdgeSet(int point_id, std::vector<int>& edges, int pre_edge, double& score, bool& inside, int& edge_id, int& start);
    
    // 更新当前GPS点和候选边集
    // inside: 该点匹配到边的内部还是端点
    // edge: 上一条匹配边
    // start_x, double start_y: 当前匹配边的起点
    // edge_set: 候选边集
    // point_id: 当前GPS点
    void UpdatePointAndCandidateEdge(bool inside, int start, std::vector<int>& edge_set, int& point_id);
    
    // 更新当前匹配边的起点
    void UpdateStart(int& start, int edge_id, int pre_edge);
    
private:
    std::vector<int> match_;
    
    boost::shared_ptr<RTree> tree_;
    boost::shared_ptr<Route> route_;
    boost::shared_ptr<ShapefileGraph> shapefile_graph_;
};

#endif