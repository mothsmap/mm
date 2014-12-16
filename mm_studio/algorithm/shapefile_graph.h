//
//  ShapefileGraph.h
//  mm
//
//  Created by Xu Xiang on 14-9-22.
//
//

#ifndef __mm__ShapefileGraph__
#define __mm__ShapefileGraph__

#include <iostream>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "rtree.h"

struct VertexProperties {
    int id_;
    int gps_id_;
    int candidate_id_;
    double location_x_, location_y_; // the coordinate of this vertex
};

struct EdgeProperties {
    int id_;
    double weight_;
};

// Use Undirected graph to represent road network for convinience
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
    ShapefileGraph() { id_max_ = 0; }
    ~ShapefileGraph() {}

    bool Build(std::string map_dir);
    bool Build(boost::shared_ptr<RTree> rtree, double minx, double miny, double maxx, double maxy);
    bool Save(std::string map_dir);
    bool Load(std::string map_dir);
    bool ComputeShortestPath(std::string map_dir);

    bool AddCandidatePoint(double sx, double sy, double ex, double ey, double cx, double cy, int gps_id, int candidate_id, int oneway, int road_id);

    inline std::vector<ShortestPath>& GetShortestPaths() { return shortest_paths_; }

    void GetPath(std::vector<Graph::edge_descriptor> edges, std::vector<BoostPoint>& points_in_path);

    void NormalizeShortestPathLengths();

    double GetMaxLength();
    
    void Reset();
private:
    bool HasVertex(double location_x, double location_y);

private:
    Graph graph_;
    int id_max_;
    std::vector<ShortestPath> shortest_paths_;
    std::vector<std::pair<int, vertex_descriptor> > candidate_vertex_;
};

#endif /* defined(__mm__ShapefileGraph__) */
