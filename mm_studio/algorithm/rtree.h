#ifndef RTREE_H__
#define RTREE_H__

#include "boost_geometry.h"
struct RoadInfo {
    //std::string name_;
    int oneway_;
};

enum QueryType {
    NODE,
    EDGE
};

class RTree {
public:
    RTree();
    ~RTree(void);
    
    bool Build(std::string map_dir);
    std::vector<Value> Query(QueryType type, double xmin, double ymin, double xmax, double ymax);
    std::vector<Value> Query(QueryType type, BoostPoint pt, int elements);
    
    inline BoostLineString GetEdge(int id) { return edges_.at(id); }
    inline BoostPoint GetNode(int id) { return nodes_.at(id); }
    inline RoadInfo GetRoadInfo(int id) { return info_.at(id); }
    
private:
    bool AddNodes(std::string map_dir);
    bool AddEdges(std::string map_dir);
private:
    Boost_RTree edge_tree_, node_tree_;
    // each edge has a unique id
    std::map<int, BoostLineString>  edges_;
    std::map<int, BoostPoint> nodes_;
    std::map<int, RoadInfo> info_;
};

#endif
