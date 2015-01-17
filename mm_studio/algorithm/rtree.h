#ifndef __rtree__hh__
#define __rtree__hh__

#include "boost_geometry.h"

// 路网的结点属性
struct VertexProperties {
    int id_;
    int gps_id_;
    int candidate_id_;
    double location_x_, location_y_;
};

// 路网边属性
struct EdgeProperties {
    int id_;
    double weight_;
    int oneway_;
    int travel_counts_;
};

// 查询类型
enum QueryType {
    NODE,
    EDGE
};

class RTree {
public:
    RTree();
    ~RTree();
    
    /************** 构建函数 *************************/
    // 给定路网的结点和边所在的目录，建立R树
    bool Build(std::string map_dir);
    
    // 插入结点
    void InsertNode(double x, double y, VertexProperties info);
    // 插入道路
    void InsertRoad(double x1, double y1, double x2, double y2, EdgeProperties info);
    
    /************** 查询函数 *************************/
    // 根据范围查询
    std::vector<Value> Query(QueryType type, double xmin, double ymin, double xmax, double ymax);
    
    // 查询点pt相邻的elements个要素
    std::vector<Value> Query(QueryType type, BoostPoint pt, int elements);
    
    /************** 获取信息 *************************/
    inline int edge_size() { return edge_count_; }
    inline int node_size() { return node_count_; }
    
    inline BoostLineString GetEdge(int id) { return edges_.at(id); }
    inline BoostPoint GetNode(int id) { return nodes_.at(id); }
    
    inline EdgeProperties GetRoadInfo(int id) { return edge_info_.at(id); }
    inline VertexProperties GetNodeInfo(int id) { return node_info_.at(id); }
    inline bool HasRoad(int id) { return edges_.find(id) != edges_.end(); }
    inline bool HasNode(int id) { return nodes_.find(id) != nodes_.end(); }
    
    inline std::vector<int>& GetEdgeNode(int id) { return edge_node_map_.at(id); }
    
    void PrintLine(int id);
    void PrintPoint(int id);
    
private:
    // Build函数的辅助函数
    bool AddNodes(std::string map_dir);
    bool AddEdges(std::string map_dir);
    
private:
    Boost_RTree edge_tree_, node_tree_;
    
    std::map<int, BoostLineString>  edges_;
    std::map<int, EdgeProperties> edge_info_;
    
    std::map<int, BoostPoint> nodes_;
    std::map<int, VertexProperties> node_info_;
    
    std::map<int, std::vector<int> > edge_node_map_;
    
    int edge_count_;
    int node_count_;
};

#endif
