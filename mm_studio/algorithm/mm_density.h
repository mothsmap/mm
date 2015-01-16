#ifndef __mm__density__hh__
#define __mm__density__hh__
#include <boost/shared_ptr.hpp>
#include <vector>
#include "boost_geometry.h"

#define VERY_SMALL_NUMBER 0.001

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
    // advance: 匹配完该点后是否前往下一个GPS点
    // start_x, start_y: 即是输入也是输出，输入表示当前匹配边的起点，输出表示下一条边的起点
    void MatchPoint2Edge(int point_id, Value& edge, double& score, bool& advance, double& start_x, double& start_y);

    // 匹配一个GPS点到候选边集中的一条边
    // point_id: GPS点的id
    // edges: 边集
    // score: 得分
    // advance: 匹配完该点后是否前往下一个GPS点
    // edge_id: 匹配到的边的id
    // start_x, start_y: 即是输入也是输出，输入表示当前匹配边的起点，输出表示下一条边的起点
    void MatchPoint2EdgeSet(int point_id, std::vector<Value>& edges, double& score, bool& advance, int& edge_id, double& start_x, double& start_y);
    
    // 更新当前GPS点和候选边集
    // advance: 是否前进一个GPS点
    // edge: 上一条匹配边
    // start_x, double start_y: 当前匹配边的起点
    // edge_set: 候选边集
    // point_id: 当前GPS点
    void UpdatePointAndCandidateEdge(bool advance, Value& edge, double start_x, double start_y, std::vector<Value>& edge_set, int& point_id);
    
    void UpdateStart(double& start_x, double& start_y, int edge_id);
    
    static bool GetProjectPoint(double x1, double y1, double x2, double y2, double x, double y, double& xx, double& yy);
    
    static bool PointEqual(double x1, double y1, double x2, double y2);
    static bool LineEqual(BoostLineString& line1, BoostLineString& line2);
    
private:
    std::vector<int> match_;
    
    boost::shared_ptr<RTree> tree_;
    boost::shared_ptr<Route> route_;
    boost::shared_ptr<ShapefileGraph> shapefile_graph_;
};

#endif