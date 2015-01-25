#ifndef __mm__graph__hh__
#define __mm__graph__hh__

#include "mm_tree.h"

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

// 使用无向图表示路网（最好是用有向图）
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

class ShapefileGraph {
public:
    ShapefileGraph(boost::shared_ptr<RTree> rtree);
    ~ShapefileGraph() {}

    // 建立图
    bool Build(double minx, double miny, double maxx, double maxy);

    inline bool HasEdgeOnVertex(int id) { return edges_on_vertex_.find(id) != edges_on_vertex_.end(); }
    
    std::vector<int> GetEdgeOnVertex(int id);
    void PrintEdgesOnVertex();
    
    // 重置图
    void Reset();
    
    void UpdateEdgeOnVertex(int vertex_id, int edge_id);
    
    void Save(std::string filename);
    void Load(std::string filename);
    
private:
    Graph graph_;
    
    boost::shared_ptr<RTree> tree_;
    
    // vertex.id_ vs id(in graph)
    std::map<int, int> vertex_;
    
    // vertex.id_ vs edge ids
    std::map<int, std::vector<int> > edges_on_vertex_;
};

#endif
