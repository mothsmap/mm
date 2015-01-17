#include "shapefile_graph.h"
#include "debug_utility.h"
#include "ogrsf_frmts.h"

ShapefileGraph::ShapefileGraph(boost::shared_ptr<RTree> rtree) {
    tree_ = rtree;
    candidate_vertex_count_ = 0;
}

void ShapefileGraph::Reset() {
    graph_.clear();
    shortest_paths_.clear();
    candidate_vertex_.clear();
    candidate_vertex_count_ = 0;
}

bool ShapefileGraph::Build(double minx, double miny, double maxx, double maxy) {
    // clean
    graph_.clear();
    shortest_paths_.clear();
    
    // build
    std::vector<Value> nodes = tree_->Query(NODE, minx, miny, maxx, maxy);
    std::vector<Value> edges = tree_->Query(EDGE, minx, miny, maxx, maxy);
    
    // 添加结点
    for (int i = 0; i < nodes.size(); ++i) {
        int geometry_id = nodes[i].second;
        BoostPoint point = tree_->GetNode(geometry_id);
        VertexProperties info = tree_->GetNodeInfo(geometry_id);
        
        Graph::vertex_descriptor vertex_descript = boost::add_vertex(graph_);
        vertex_.insert(std::make_pair(info.id_, vertex_descript));
        
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

void ShapefileGraph::PrintEdgesOnVertex() {
    for (auto iter = edges_on_vertex_.begin(); iter != edges_on_vertex_.end(); ++iter) {
        std::cout << "Node " << iter->first << " has edges: ";
        for (int i = 0; i < iter->second.size(); ++i) {
            std::cout << iter->second[i] << "\t";
        }
        std::cout << std::endl;
    }
}

bool ShapefileGraph::AddCandidatePoint(double cx, double cy, int gps_id, int candidate_id, EdgeProperties edge) {
    if (!tree_->HasRoad(edge.id_)) {
        DebugUtility::Print(DebugUtility::Warning, "No road in graph!");
        return false;
    }
    
    BoostLineString line = tree_->GetEdge(edge.id_);
    double sx = line.at(0).get<0>();
    double sy = line.at(0).get<1>();
    double ex = line.at(1).get<0>();
    double ey = line.at(1).get<1>();
    
    std::vector<int>& node_index = tree_->GetEdgeNode(edge.id_);
    Graph::vertex_descriptor start_vertex = vertex_.at(node_index[0]);
    Graph::vertex_descriptor end_vertex = vertex_.at(node_index[1]);
    
    // 得到两个节点组成的边
    std::pair<Graph::edge_descriptor, bool> original_edge_pair = boost::edge(start_vertex, end_vertex, graph_);
    
    if (!original_edge_pair.second) {
        DebugUtility::Print(DebugUtility::Warning, "No road in graph!");
        return false;
    }
    
    if (edge.id_ != graph_[original_edge_pair.first].id_); {
        DebugUtility::Print(DebugUtility::Warning, "Road is insistent!");
        return false;
    }
    
    // 插入这个候选点
    Graph::vertex_descriptor vertex_descript = boost::add_vertex(graph_);
    graph_[vertex_descript].id_ = tree_->node_size() + candidate_vertex_count_;
    graph_[vertex_descript].gps_id_ = gps_id;
    graph_[vertex_descript].candidate_id_ = candidate_id;
    graph_[vertex_descript].location_x_ = cx;
    graph_[vertex_descript].location_y_ = cy;
    ++candidate_vertex_count_;
    
    // 开始节点到候选点的边
    std::pair<Graph::edge_descriptor, bool> edge_pair = boost::add_edge(start_vertex, vertex_descript, graph_);
    graph_[edge_pair.first].id_ = graph_[original_edge_pair.first].id_;
    graph_[edge_pair.first].weight_ = GeometryUtility::Distance(sx, sy, cx, cy);
    
    // 候选点到终止节点的边
    edge_pair = boost::add_edge(vertex_descript, end_vertex, graph_);
    graph_[edge_pair.first].id_ = graph_[original_edge_pair.first].id_;
    graph_[edge_pair.first].weight_ = GeometryUtility::Distance(ex, ey, cx, cy);
    
    // 候选点到其它候选点的边（如果有的话）
    for (int i = 0; i < candidate_vertex_.size(); ++i) {
        std::pair<int, vertex_descriptor> vertex = candidate_vertex_[i];
        // 如果之前的某个候选点和当前候选点在同一条路上的话
        if (vertex.first == edge.id_) {
            edge_pair = boost::add_edge(vertex.second, vertex_descript, graph_);
            graph_[edge_pair.first].id_ = edge.id_;
            graph_[edge_pair.first].weight_ = GeometryUtility::Distance(graph_[vertex.second].location_x_, graph_[vertex.second].location_y_, graph_[vertex_descript].location_x_, graph_[vertex_descript].location_y_);
        }
    }
    
    // 把当前候选点添加到已添加候选点的列表中
    candidate_vertex_.push_back(std::make_pair(edge.id_, vertex_descript));
    
    return true;
}

bool ShapefileGraph::ComputeShortestPath() {
    DebugUtility::Print(DebugUtility::Normal, "Compute the shortest path ...");
    
    std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vertex_iterator_range = boost::vertices(graph_);
    for (Graph::vertex_iterator from_vertex = vertex_iterator_range.first; from_vertex != vertex_iterator_range.second; ++from_vertex) {
        // We only caculate gps vertex
        if (graph_[*from_vertex].gps_id_ == -1)
            continue;
        
        // Create things for Dijkstra
        std::vector<vertex_descriptor> parents(boost::num_vertices(graph_)); // To store parents
        std::vector<double> distances(boost::num_vertices(graph_)); // To store distances
        
        // Compute shortest paths from v0 to all vertices, and store the output in parents and distances
        /*
         boost::dijkstra_shortest_paths(graph_,
         *(from_vertex),
         boost::predecessor_map(&parents[0])
         .distance_map(&distances[0])
         .weight_map(get(&EdgeProperties::weight_, graph_)));*/
        boost::dijkstra_shortest_paths(graph_,
                                       *(from_vertex),
                                       boost::predecessor_map(&parents[0])
                                       .distance_map(boost::make_iterator_property_map(distances.begin(),
                                       get(boost::vertex_index, graph_)))
                                       .weight_map(get(&EdgeProperties::weight_, graph_)));

        for (Graph::vertex_iterator to_vertex = vertex_iterator_range.first; to_vertex != vertex_iterator_range.second; ++to_vertex) {
            if (graph_[*to_vertex].gps_id_ != (graph_[*from_vertex].gps_id_ + 1))
                continue;
            
            bool path_valid = true;
            
            ShortestPath shortest_path;
            // id
            shortest_path.from_vertex_id_ = graph_[*from_vertex].id_;
            
            shortest_path.to_vertex_id_ = graph_[*to_vertex].id_;
            
            // candidate id
            shortest_path.from_vertex_candidate_id_ = graph_[*from_vertex].candidate_id_;
            
            shortest_path.to_vertex_candidate_id_ = graph_[*to_vertex].candidate_id_;
            
            // gps id
            shortest_path.from_vertex_gps_id_ = graph_[*from_vertex].gps_id_;
            
            shortest_path.to_vertex_gps_id_ = graph_[*to_vertex].gps_id_;
            
            // location
            shortest_path.from_vertex_location_x_ = graph_[*from_vertex].location_x_;
            shortest_path.from_vertex_location_y_ = graph_[*from_vertex].location_y_;
            shortest_path.to_vertex_location_x_ = graph_[*to_vertex].location_x_;
            shortest_path.to_vertex_location_y_ = graph_[*to_vertex].location_y_;
            
            // length
            shortest_path.length_ = distances[*to_vertex];
            
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
                
                shortest_path.path_.push_back(graph_[edge].id_);
                
                // fetch vertexs from edge
                ////////////////
                vertex_descriptor source = boost::source(edge, graph_);
                vertex_descriptor target = boost::target(edge, graph_);
                if (vertex_.find(graph_[source].id_) != vertex_.end() && vertex_.find(graph_[target].id_) != vertex_.end()) {
                    shortest_path.nodes_.push_back(BoostPoint(graph_[source].location_x_, graph_[source].location_y_));
                    shortest_path.nodes_.push_back(BoostPoint(graph_[target].location_x_, graph_[target].location_y_));
                } else {
                    DebugUtility::Print(DebugUtility::Error, "Not find source and target together!");
                }
            }
            
            if (path_valid) {
                shortest_paths_.push_back(shortest_path);
            }
        }
    }
    
    return shortest_paths_.size() > 0;
}

void ShapefileGraph::PrintShortestPath() {
    for (int i = 0; i < shortest_paths_.size(); ++i) {
        ShortestPath path = shortest_paths_[i];
        std::cout << "From [" << path.from_vertex_id_ << "," << path.from_vertex_gps_id_ << ", " << path.from_vertex_candidate_id_ << "] to [" << path.to_vertex_id_ << "," << path.to_vertex_gps_id_ << "," << path.to_vertex_candidate_id_ << "] length is : " << path.length_ << std::endl;
        
        if (path.path_.size() == 0) {
            std::cout << "path is empty." << std::endl;
            
            continue;
        }
        
        std::cout << "path: ";
        for (int j = path.path_.size() - 1; j > 0; --j) {
            std::cout << path.path_[j] << " -> ";
        }
        std::cout << path.path_[0] << std::endl;
    }
}