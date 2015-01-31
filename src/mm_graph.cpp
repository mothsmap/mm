#include "debug.h"
#include "mm_graph.h"
#include <fstream>
#include "ogrsf_frmts.h"

ShapefileGraph::ShapefileGraph(boost::shared_ptr<RTree> rtree) {
    tree_ = rtree;
}

void ShapefileGraph::Reset() {
    graph_.clear();
}

bool ShapefileGraph::Build(double minx, double miny, double maxx, double maxy) {
    // clean
    graph_.clear();
    
    // build
    std::vector<Value> nodes = tree_->Query(NODE, minx, miny, maxx, maxy);
    std::vector<Value> edges = tree_->Query(EDGE, minx, miny, maxx, maxy);
    
    // 添加结点
    for (int i = 0; i < nodes.size(); ++i) {
        int geometry_id = nodes[i].second;
        BoostPoint point = tree_->GetNode(geometry_id);
        VertexProperties info = tree_->GetNodeInfo(geometry_id);
        
        Graph::vertex_descriptor vertex_descript = boost::add_vertex(graph_);
        vertex_.insert(std::make_pair(info.id_, i));
        
        graph_[vertex_descript].id_ = info.id_;
        graph_[vertex_descript].candidate_id_ = info.candidate_id_;
        graph_[vertex_descript].gps_id_ = info.gps_id_;
        graph_[vertex_descript].location_x_ = point.get<0>();
        graph_[vertex_descript].location_y_ = point.get<1>();
    }
    
    // 添加边
    for (int i = 0; i < edges.size(); ++i) {
        int geometry_id = edges[i].second;
        BoostLineString line_string = tree_->GetEdge(geometry_id);
        //EdgeProperties info = rtree->GetRoadInfo(geometry_id);
        
        double x1 = line_string.at(0).get<0>();
        double y1 = line_string.at(0).get<1>();
        double x2 = line_string.at(1).get<0>();
        double y2 = line_string.at(1).get<1>();
        
        std::vector<int>& node_index = tree_->GetEdgeNode(geometry_id);
        Graph::vertex_descriptor start_vertex = vertex_.at(node_index[0]);
        Graph::vertex_descriptor end_vertex = vertex_.at(node_index[1]);
        
        std::pair<Graph::edge_descriptor, bool> edge_pair = boost::add_edge(start_vertex, end_vertex, graph_);
        graph_[edge_pair.first].id_ = geometry_id;
        graph_[edge_pair.first].weight_ = GeometryUtility::Distance(x1, y1, x2, y2);
        
        this->UpdateEdgeOnVertex(node_index[0], geometry_id);
        this->UpdateEdgeOnVertex(node_index[1], geometry_id);
    }
    
    return true;
}

void ShapefileGraph::UpdateEdgeOnVertex(int vertex_id, int edge_id) {
    if (edges_on_vertex_.find(vertex_id) == edges_on_vertex_.end()) {
        std::vector<int> edges;
        edges.push_back(edge_id);
        edges_on_vertex_.insert(std::make_pair(vertex_id, edges));
    } else {
        std::vector<int> edges = edges_on_vertex_.at(vertex_id);
        edges.push_back(edge_id);
        edges_on_vertex_.erase(edges_on_vertex_.find(vertex_id));
        edges_on_vertex_.insert(std::make_pair(vertex_id, edges));
    }
}

bool ShapefileGraph::ShortestPath(int node1, int node2, std::vector<int>& path, double& dist) {
    DebugUtility::Print(DebugUtility::Normal, "Compute the shortest path ...");
    bool path_valid = true;
    
    std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vertex_iterator_range = boost::vertices(graph_);
    for (Graph::vertex_iterator from_vertex = vertex_iterator_range.first; from_vertex != vertex_iterator_range.second; ++from_vertex) {
        if (graph_[*from_vertex].id_ != node1)
            continue;
        
        // Create things for Dijkstra
        std::vector<vertex_descriptor> parents(boost::num_vertices(graph_)); // To store parents
        std::vector<double> distances(boost::num_vertices(graph_)); // To store distances
        
        boost::dijkstra_shortest_paths(graph_,
                                       *(from_vertex),
                                       boost::predecessor_map(&parents[0])
                                       .distance_map(boost::make_iterator_property_map(distances.begin(),
                                                                                       get(boost::vertex_index, graph_)))
                                       .weight_map(get(&EdgeProperties::weight_, graph_)));
        
        for (Graph::vertex_iterator to_vertex = vertex_iterator_range.first; to_vertex != vertex_iterator_range.second; ++to_vertex) {
            if (graph_[*to_vertex].id_ != node2) {
                continue;
            }
            
            dist = distances[*to_vertex];
            
            vertex_descriptor v = *to_vertex; // We want to start at the destination and work our way back to the source
            for(vertex_descriptor u = parents[v]; // Start by setting 'u' to the destintaion node's predecessor
                u != v; // Keep tracking the path until we get to the source
                v = u, u = parents[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
            {
                std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, graph_);
                Graph::edge_descriptor edge = edgePair.first;
                
                if (!edgePair.second) {
                    path_valid = false;
                    break;
                }
                path.push_back(graph_[edge].id_);
            }
        }
        
    }
    
    return path_valid;
}

void ShapefileGraph::PrintEdgesOnVertex() {
    for (auto iter = edges_on_vertex_.begin(); iter != edges_on_vertex_.end(); ++iter) {
        std::cout << "Node " << iter->first << " has edges: ";
        for (int i = 0; i < iter->second.size(); ++i) {
            std::cout << iter->second[i] << "\t";
        }
        std::cout << std::endl;
    }
}

void ShapefileGraph::Save(std::string filename) {
    std::ofstream ofs(filename);
    boost::archive::xml_oarchive oa(ofs);
    boost::serialization::save(oa, graph_, 0);
    
    oa & BOOST_SERIALIZATION_NVP(vertex_);
    oa & BOOST_SERIALIZATION_NVP(edges_on_vertex_);
    
    ofs.close();
}

void ShapefileGraph::Load(std::string filename) {
    DebugUtility::Print(DebugUtility::Normal, "Loading graph...");
    std::ifstream ifs(filename);
    boost::archive::xml_iarchive ia(ifs);
    boost::serialization::load(ia, graph_, 0);
    
    ia & BOOST_SERIALIZATION_NVP(vertex_);
    ia & BOOST_SERIALIZATION_NVP(edges_on_vertex_);
    
    DebugUtility::Print(DebugUtility::Verbose, "Loading " + boost::lexical_cast<std::string>(vertex_.size()) + " vertex.");
    
    ifs.close();
}

std::vector<int> ShapefileGraph::GetEdgeOnVertex(int id) {
    std::vector<int> out_edges;
    
    Graph::out_edge_iterator edgeIt, edgeEnd;
    
    tie(edgeIt, edgeEnd) = boost::out_edges(vertex_.at(id), graph_);
    
    for(; edgeIt != edgeEnd; ++edgeIt) {
        out_edges.push_back(graph_[*edgeIt].id_);
    }
    
    return out_edges;
}