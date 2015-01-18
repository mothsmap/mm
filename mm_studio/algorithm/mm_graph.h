#ifndef __mm__graph__hh__
#define __mm__graph__hh__

#include "mm_tree.h"

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>

// 使用无向图表示路网（最好是用有向图）
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

struct ShortestPath {
    // the id in graph
    int from_vertex_id_, to_vertex_id_;
    // the id in gps points
    int from_vertex_gps_id_, to_vertex_gps_id_;
    // the id in candidate group
    int from_vertex_candidate_id_, to_vertex_candidate_id_;
    
    // position
    double from_vertex_location_x_, from_vertex_location_y_, to_vertex_location_x_, to_vertex_location_y_;

    // length
    double length_;
    
    // road segments id
    std::vector<int> path_;
    
    // vertices
    std::vector<BoostPoint> nodes_;

    bool operator== (const ShortestPath& path) const {
        return (this->from_vertex_id_ == this->to_vertex_id_);
    }

    bool operator!= (const ShortestPath& path) const {
        return (this->from_vertex_id_ != this->to_vertex_id_);
    }

    bool operator> (const ShortestPath& path) const{
        if(from_vertex_id_ == path.from_vertex_id_)
            return (to_vertex_id_ > path.to_vertex_id_);
        else
            return (from_vertex_id_ > path.from_vertex_id_);
    }

    bool operator< (const ShortestPath& path) const {
        if(from_vertex_id_ == path.from_vertex_id_)
            return (to_vertex_id_ < path.to_vertex_id_);
        else
            return (from_vertex_id_ < path.from_vertex_id_);
    }
};

class ShapefileGraph {
public:
    ShapefileGraph(boost::shared_ptr<RTree> rtree);
    ~ShapefileGraph() {}

    // 建立图
    bool Build(double minx, double miny, double maxx, double maxy);
    
    // 添加候选点
    bool AddCandidatePoint(double cx, double cy, int gps_id, int candidate_id, EdgeProperties edge);
    
    // 计算候选点之间的最短路径
    bool ComputeShortestPath();

    // 得到最短路径集合
    inline std::vector<ShortestPath>& GetShortestPaths() { return shortest_paths_; }

    inline bool HasEdgeOnVertex(int id) { return edges_on_vertex_.find(id) != edges_on_vertex_.end(); }
    inline std::vector<int>& GetEdgeOnVertex(int id) { return edges_on_vertex_.at(id); }
    
    // 重置图
    void Reset();
    
    void UpdateEdgeOnVertex(int vertex_id, int edge_id);
    
    void PrintShortestPath();
    void PrintEdgesOnVertex();
private:
    Graph graph_;
    
    boost::shared_ptr<RTree> tree_;
    
    std::map<int, vertex_descriptor> vertex_;
    std::map<int, std::vector<int> > edges_on_vertex_;
    
    std::vector<ShortestPath> shortest_paths_;
    std::vector<std::pair<int, vertex_descriptor> > candidate_vertex_;
    int candidate_vertex_count_;
};

#endif
