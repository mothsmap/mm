//
//  ShapefileGraph.cpp
//  mm
//
//  Created by Xu Xiang on 14-9-22.
//
//

#include "shapefile_graph.h"
#include "ogrsf_frmts.h"
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

const double thd = 0.001;
bool PointEqual(double x1, double y1, double x2, double y2, const double thd) {
    double dist_x = x1 - x2;
    double dist_y = y1 - y2;
    
    double dist = dist_x * dist_x + dist_y * dist_y;
    
    return (dist < thd);
}

double Distance(double x1, double y1, double x2, double y2) {
    double dist_x = x1 - x2;
    double dist_y = y1 - y2;
    
    double dist = dist_x * dist_x + dist_y * dist_y;
    
    return std::sqrt(dist);
}

void ShapefileGraph::Reset() {
    graph_.clear();
    id_max_ = 0;
    shortest_paths_.clear();
    candidate_vertex_.clear();
}

bool ShapefileGraph::Build(boost::shared_ptr<RTree> rtree, double minx, double miny, double maxx, double maxy) {
    // clean
    graph_.clear();
    id_max_ = 0;
    shortest_paths_.clear();
    
    // build
    std::vector<Value> nodes = rtree->Query(NODE, minx, miny, maxx, maxy);
    std::vector<Value> edges = rtree->Query(EDGE, minx, miny, maxx, maxy);
    
    // First add vertices to graph
    int nodes_size = nodes.size();
#if PRINT_INFO
    std::cout << "Add " << nodes_size << " nodes.\n";
#endif
    for (int i = 0; i < nodes_size; ++i) {
        int geometry_id = nodes[i].second;
        BoostPoint point = rtree->GetNode(geometry_id);
        
        Graph::vertex_descriptor vertex_descript = boost::add_vertex(graph_);
        graph_[vertex_descript].id_ = id_max_++;
        graph_[vertex_descript].candidate_id_ = -1;
        graph_[vertex_descript].gps_id_ = -1;
        graph_[vertex_descript].location_x_ = point.get<0>();
        graph_[vertex_descript].location_y_ = point.get<1>();
    }
    
    // Then add edges
    int edge_size = edges.size();
#if PRINT_INFO
    std::cout << "Add " << edge_size << " edges.\n";
#endif
    int edge_added_count = 0;
    for (int i = 0; i < edge_size; ++i) {
        int geometry_id = edges[i].second;
        BoostLineString line_string = rtree->GetEdge(geometry_id);
        RoadInfo info = rtree->GetRoadInfo(geometry_id);
        double x1 = line_string.at(0).get<0>();
        double y1 = line_string.at(0).get<1>();
        double x2 = line_string.at(1).get<0>();
        double y2 = line_string.at(1).get<1>();
        
        // Check the nodes of this edge
        bool find_first = false;
        bool find_second = false;
        std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vertex_iterator_range = boost::vertices(graph_);
        Graph::vertex_iterator start_vertex_iterator, end_vertex_iterator;
        for (Graph::vertex_iterator vertex_iterator = vertex_iterator_range.first; vertex_iterator != vertex_iterator_range.second; ++vertex_iterator) {
            if (!find_first) {
                bool equal_test = PointEqual(x1, y1, graph_[*vertex_iterator].location_x_, graph_[*vertex_iterator].location_y_, thd);
                if (equal_test) {
                    find_first = true;
                    start_vertex_iterator = vertex_iterator;
                }
            }
            if (!find_second) {
                bool equal_test = PointEqual(x2, y2, graph_[*vertex_iterator].location_x_, graph_[*vertex_iterator].location_y_, thd);
                if (equal_test) {
                    find_second = true;
                    end_vertex_iterator = vertex_iterator;
                }
            }
            if (find_first && find_second)
                break;
        }
        
        // Find the nodes of this edge
        if (find_first && find_second) {
            // Add a edge
            std::pair<Graph::edge_descriptor, bool> edge_pair = boost::add_edge(*start_vertex_iterator, *end_vertex_iterator, graph_);
            graph_[edge_pair.first].id_ = geometry_id;
            graph_[edge_pair.first].weight_ = Distance(x1, y1, x2, y2);
            
            edge_added_count++;
        }
        
    }
    
#if PRINT_INFO
    std::cout << "Add " << edge_added_count << " edges in fact.\n";
    std::cout << "Graph building finished.\n";
#endif
    
    return true;
}

bool ShapefileGraph::Save(std::string map_dir) {
    return false;
}

bool ShapefileGraph::Load(std::string map_dir) {
    return false;
}

bool ShapefileGraph::ComputeShortestPath(std::string map_dir) {
#if PRINT_INFO
    std::cout << "\nCompute the shortest path ...\n";
#endif
    std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vertex_iterator_range = boost::vertices(graph_);
    
    for (Graph::vertex_iterator from_vertex = vertex_iterator_range.first; from_vertex != vertex_iterator_range.second; ++from_vertex) {
        // We only caculate gps vertex
        if (graph_[*from_vertex].gps_id_ == -1)
            continue;
        
        // Create things for Dijkstra
        std::vector<vertex_descriptor> parents(boost::num_vertices(graph_)); // To store parents
        std::vector<double> distances(boost::num_vertices(graph_)); // To store distances
        
        // Compute shortest paths from v0 to all vertices, and store the output in parents and distances
        boost::dijkstra_shortest_paths(graph_,
                                       *(from_vertex),
                                       boost::predecessor_map(&parents[0])
                                       .distance_map(&distances[0])
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
                int source = boost::source(edge, graph_);
                int target = boost::target(edge, graph_);
                
                bool find_source = false;
                bool find_target = false;
                Graph::vertex_iterator start_vertex_iterator, end_vertex_iterator;
                
                for (Graph::vertex_iterator vertex_iterator = vertex_iterator_range.first; vertex_iterator != vertex_iterator_range.second; ++vertex_iterator) { // 4
                    if(graph_[*vertex_iterator].id_ == source) {
                        find_source = true;
                        start_vertex_iterator = vertex_iterator;
                    }
                    if (graph_[*vertex_iterator].id_ == target) {
                        find_target = true;
                        end_vertex_iterator = vertex_iterator;
                    }
                    if (find_source && find_target) {
                        break;
                    }
                } // 4
                
                if (find_source && find_target) {
                    shortest_path.nodes_.push_back(BoostPoint(graph_[*start_vertex_iterator].location_x_,
                                                        graph_[*start_vertex_iterator].location_y_));
                    shortest_path.nodes_.push_back(BoostPoint(graph_[*end_vertex_iterator].location_x_,
                                                        graph_[*end_vertex_iterator].location_y_));
                } else {
                    std::cout << "Error: Not find source and target together.\n";
                }
                ////////////////
            }
#if PRINT_INFO
            std::cout << "Add a shortest path\n";
#endif
            if (path_valid) {
                shortest_paths_.push_back(shortest_path);
            }
        }
        
    }
#if PRINT_INFO
    // std::cout << "\nSave the shortest path ...\n";
    std::string file = map_dir + "/sp.txt";
    std::ofstream ofs(file.c_str());
    for (int i = 0; i < shortest_paths_.size(); ++i) {
        ShortestPath path = shortest_paths_[i];
        ofs << "From [" << path.from_vertex_id_ << "," << path.from_vertex_gps_id_ << ", " << path.from_vertex_candidate_id_ << "] to [" << path.to_vertex_id_ << "," << path.to_vertex_gps_id_ << "," << path.to_vertex_candidate_id_ << "] length is : " << path.length_ << std::endl;
        
        if (path.path_.size() == 0) {
            ofs << "path is empty." << std::endl;
            
            continue;
        }
        
        ofs << "path: ";
        for (int j = path.path_.size() - 1; j > 0; --j) {
            ofs << path.path_[j] << " -> ";
        }
        ofs << path.path_[0] << std::endl;
    }
    
    ofs.close();
#endif
    return shortest_paths_.size() > 0;
}

double ShapefileGraph::GetMaxLength() {
    double max_length =  shortest_paths_[0].length_;
    for (int i = 1; i < shortest_paths_.size(); ++i) {
        if (max_length < shortest_paths_[i].length_)
            max_length = shortest_paths_[i].length_;
    }
    return max_length;
}

void ShapefileGraph::NormalizeShortestPathLengths() {
    double max_length =  shortest_paths_[0].length_;
    double min_length = max_length;
    
    for (int i = 1; i < shortest_paths_.size(); ++i) {
        if (max_length < shortest_paths_[i].length_)
            max_length = shortest_paths_[i].length_;
        if (min_length > shortest_paths_[i].length_)
            min_length = shortest_paths_[i].length_;
    }
    
    for (int i = 0; i < shortest_paths_.size(); ++i) {
        shortest_paths_[i].length_ = (shortest_paths_[i].length_ - min_length) / (max_length - min_length);
    }
}

bool ShapefileGraph::AddCandidatePoint(double sx, double sy, double ex, double ey, double cx, double cy, int gps_id, int candidate_id, int oneway, int road_id) {
#if PRINT_INFO
    std::cout << "插入候选点到图种...\n";
#endif
    // 寻找候选点所在边的两个节点
    std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vertex_iterator_range = boost::vertices(graph_);
    Graph::vertex_iterator start_vertex_iterator, end_vertex_iterator;
    bool find_first = false;
    bool find_second = false;
    for (Graph::vertex_iterator vertex_iterator = vertex_iterator_range.first; vertex_iterator != vertex_iterator_range.second; ++vertex_iterator) {
        if (!find_first) {
            bool equal_test = PointEqual(sx, sy, graph_[*vertex_iterator].location_x_, graph_[*vertex_iterator].location_y_, thd);
            if (equal_test) {
                find_first = true;
                start_vertex_iterator = vertex_iterator;
            }
        }
        if (!find_second) {
            bool equal_test = PointEqual(ex, ey, graph_[*vertex_iterator].location_x_, graph_[*vertex_iterator].location_y_, thd);
            if (equal_test) {
                find_second = true;
                end_vertex_iterator = vertex_iterator;
            }
        }
        if (find_first && find_second)
            break;
    }
    
    // 找到了节点
    if (find_first && find_second) {
        // 得到两个节点组成的边
        std::pair<Graph::edge_descriptor, bool> original_edge_pair = boost::edge(*start_vertex_iterator, *end_vertex_iterator, graph_);
        
        if (!original_edge_pair.second)
            return false;
        assert(road_id == graph_[original_edge_pair.first].id_);
        
        // 插入这个候选点
        Graph::vertex_descriptor vertex_descript = boost::add_vertex(graph_);
        graph_[vertex_descript].id_ = id_max_++;
        graph_[vertex_descript].gps_id_ = gps_id;
        graph_[vertex_descript].candidate_id_ = candidate_id;
        graph_[vertex_descript].location_x_ = cx;
        graph_[vertex_descript].location_y_ = cy;
        
        // 开始节点到候选点的边
        std::pair<Graph::edge_descriptor, bool> edge_pair = boost::add_edge(*start_vertex_iterator, vertex_descript, graph_);
        graph_[edge_pair.first].id_ = graph_[original_edge_pair.first].id_;
        graph_[edge_pair.first].weight_ = Distance(sx, sy, cx, cy);
        
        // 候选点到终止节点的边
        edge_pair = boost::add_edge(vertex_descript, *end_vertex_iterator, graph_);
        graph_[edge_pair.first].id_ = graph_[original_edge_pair.first].id_;
        graph_[edge_pair.first].weight_ = Distance(ex, ey, cx, cy);
        
        // 候选点到其它候选点的边（如果有的话）
        for (int i = 0; i < candidate_vertex_.size(); ++i) {
            std::pair<int, vertex_descriptor> vertex = candidate_vertex_[i];
            // 如果之前的某个候选点和当前候选点在同一条路上的话
            if (vertex.first == road_id) {
                edge_pair = boost::add_edge(vertex.second, vertex_descript, graph_);
                graph_[edge_pair.first].id_ = road_id;
                graph_[edge_pair.first].weight_ = Distance(graph_[vertex.second].location_x_, graph_[vertex.second].location_y_, graph_[vertex_descript].location_x_, graph_[vertex_descript].location_y_);
            }
        }
        
        // 把当前候选点添加到已添加候选点的列表中
        candidate_vertex_.push_back(std::make_pair(road_id, vertex_descript));
        
        // 返回
#if PRINT_INFO
        std::cout << "success!\n";
#endif
        return true;
    }
    
    return false;
}

void ShapefileGraph::GetPath(std::vector<Graph::edge_descriptor> edges, std::vector<BoostPoint>& points_in_path) { // 0
    std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vertex_iterator_range = boost::vertices(graph_);
    Graph::vertex_iterator start_vertex_iterator, end_vertex_iterator;
    
    // 提取顶点
    for (int j = edges.size() - 1; j > 0; --j) { // 3
        edge_descriptor edge_des = edges[j];
        int source = boost::source(edge_des, graph_);
        int target = boost::target(edge_des, graph_);
        
        bool find_source = false;
        bool find_target = false;
        for (Graph::vertex_iterator vertex_iterator = vertex_iterator_range.first; vertex_iterator != vertex_iterator_range.second; ++vertex_iterator) { // 4
            if(graph_[*vertex_iterator].id_ == source) {
                find_source = true;
                start_vertex_iterator = vertex_iterator;
            }
            if (graph_[*vertex_iterator].id_ == target) {
                find_target = true;
                end_vertex_iterator = vertex_iterator;
            }
            if (find_source && find_target) {
                break;
            }
        } // 4
        
        if (find_source && find_target) {
            points_in_path.push_back(BoostPoint(graph_[*start_vertex_iterator].location_x_,
                                                graph_[*start_vertex_iterator].location_y_));
            points_in_path.push_back(BoostPoint(graph_[*end_vertex_iterator].location_x_,
                                                graph_[*end_vertex_iterator].location_y_));
        } else {
            std::cout << "Error: Not find source and target together.\n";
        }
    } // 3
} // 0