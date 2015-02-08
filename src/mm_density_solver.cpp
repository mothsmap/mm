#include <boost/lexical_cast.hpp>
#include "debug.h"
#include "mm_density_solver.h"
#include "mm_graph.h"
#include "mm_route.h"
#include "mm_tree.h"

MMDensity::MMDensity(boost::shared_ptr<RTree> rtree, boost::shared_ptr<ShapefileGraph> shapefile_graph) {
    tree_ = rtree;
    shapefile_graph_ = shapefile_graph;
    has_error_ = false;
    dist_parameter_ = 150;
}

MMDensity::~MMDensity() {
    
}

void MMDensity::Match() {
    try {
        int ntrajectories = tree_->trajectory_size();
        for (int i = 0; i < ntrajectories; ++i) {
            std::vector<int> matched = Match(i);
            if (!has_error_ && matched.size() > 0 ) {
                tree_->InsertMatchedTrajectory(matched, i);
            } else {
                DebugUtility::Print(DebugUtility::Error, "Miss history trajectory " + boost::lexical_cast<std::string>(i));
                has_error_ = false;
            }
        }
    } catch (std::exception e) {
        DebugUtility::Print(DebugUtility::Error, "An exception " + boost::lexical_cast<std::string>(e.what()));
    } catch (...) {
        DebugUtility::Print(DebugUtility::Error, "An exception occurs!");
    }
}

std::vector<int> MMDensity::Match(int trajectory_id) {
    // 匹配结果
    std::vector<int> matched;
    
    // GPS轨迹
    match_id_ = trajectory_id;
    GPSTrajectory& trajectory = tree_->GetTrajectory(trajectory_id);
    DebugUtility::Print(DebugUtility::Normal, "match " + boost::lexical_cast<std::string>(trajectory.size()) + " GPS points");
    if (trajectory.size() == 0) {
        DebugUtility::Print(DebugUtility::Error, "GPS trajecotry is empty!");
        return matched;
    }
    
    // 当前gps点
    int current_gps = 0;
    
    // 初始化：当前GPS点邻近的边集
    std::vector<int> edge_set;
    
    std::vector<Value> edge_value_set = tree_->Query(EDGE, trajectory[0].x_ - dist_parameter_, trajectory[0].y_ - dist_parameter_, trajectory[0].x_ + dist_parameter_, trajectory[0].y_ + dist_parameter_);
    
    if (edge_value_set.size() == 0) {
        DebugUtility::Print(DebugUtility::Error, "Initialize edge set fail!");
        std::cout << "(x, y, t) = (" << trajectory[0].x_ << ", " << trajectory[0].y_ << ", " << trajectory[0].t_ << ")\n";
        has_error_ = true;
        return matched;
    } else {
        DebugUtility::Print(DebugUtility::Normal, "Initialize edge set size = " + boost::lexical_cast<std::string>(edge_value_set.size()));
        
        for (int i = 0; i < edge_value_set.size(); ++i) {
            edge_set.push_back(edge_value_set[i].second);
        }
    }
    
    // 前一条匹配边
    int pre_edge_outer = -1;
    
    // 当前边的起点
    int start_outer = -1;
    int start_outer2 = -1;
    
    // 是否要前往下个GPS点
    bool advance_outer;
    
    int repeat_count = 0;
    
    int pre_gps;
    
    while (current_gps < trajectory.size()) {
        if (pre_gps == current_gps) {
            repeat_count++;
        } else {
            repeat_count = 0;
        }
        
        if (repeat_count > 2000) {
            std::cout << "Solve a dead loop!\n";
            current_gps++;
            repeat_count = 0;
//            has_error_ = true;
//            return matched;
        }
        
        pre_gps = current_gps;
        
        DebugUtility::Print(DebugUtility::Normal, "GPS #" + boost::lexical_cast<std::string>(current_gps) + " has " + boost::lexical_cast<std::string>(edge_set.size()) + " candidate edges\n" + "Pre edge " + boost::lexical_cast<std::string>(pre_edge_outer) + "\n" +
                            "start vertex " + boost::lexical_cast<std::string>(start_outer));
        
        if (edge_set.size() == 0) {
            DebugUtility::Print(DebugUtility::Warning, "Candidate edge set empty!");
            return matched;
        }
        
        // 当前GPS点匹配的最佳得分
        double best_score = std::numeric_limits<double>::min();
        
        // 当前GPS点最佳匹配边
        int best_edge_id = -1;
        
        // 循环处理当前GPS点的候选边集，寻找最佳边
        for (int i = 0; i < edge_set.size(); ++i) {
            int start = start_outer;
            int pre_edge = pre_edge_outer;
            
            DebugUtility::Print(DebugUtility::Normal, "Processing edge " + boost::lexical_cast<std::string>(edge_set[i]) + ", start = " + boost::lexical_cast<std::string>(start));
            
            // 首先将该GPS点匹配到当前边
            bool advance;
            double score;
            if (!this->MatchPoint2Edge(current_gps, edge_set[i], pre_edge, score, advance, start)) {
                has_error_ = true;
                return matched;
            }
            
            DebugUtility::Print(DebugUtility::Normal, "\t Match GPS point " + boost::lexical_cast<std::string>(current_gps) + " to edge... score = " + boost::lexical_cast<std::string>(score) + ", start = " + boost::lexical_cast<std::string>(start));
            
            // 往前探索
            int pre_edge_inside = edge_set[i];
            int current_gps_inside = current_gps;
            int start_inside = start;
            bool advance_inside = advance;
           
            int step = 0;
            while (step <= 3 && current_gps < trajectory.size()) {
                step++;
                
                DebugUtility::Print(DebugUtility::Normal, "\t\t Advance step " + boost::lexical_cast<std::string>(step));
                
                double score_inside;
                std::vector<int> edge_set_inside;
                int edge_inside;
                
                this->UpdatePointAndCandidateEdge(advance_inside, start_inside, pre_edge_inside, edge_set_inside, current_gps_inside);
                
                if (edge_set_inside.size() == 0) {
                    score += -10000;
                    break;
                }
                
                if (current_gps_inside >= trajectory.size()) {
                    DebugUtility::Print(DebugUtility::Warning, "Reach the end of GPS point");
                    score += 10.0;
                    break;
                }
                
                DebugUtility::Print(DebugUtility::Normal, "\t\t Update GPS point and Candidate edge set... current gps = " + boost::lexical_cast<std::string>(current_gps_inside) + ", edge set size = " + boost::lexical_cast<std::string>(edge_set_inside.size()));
                
                
                if(!this->MatchPoint2EdgeSet(current_gps_inside, edge_set_inside, pre_edge_inside, score_inside, advance_inside, edge_inside, start_inside)) {
                    has_error_ = true;
                    return matched;
                }
                
                DebugUtility::Print(DebugUtility::Normal, "\t\t Match GPS point and Candidate edge set... score = " + boost::lexical_cast<std::string>(score_inside) + ", matched edge = " + boost::lexical_cast<std::string>(edge_inside) + ", start inside = " + boost::lexical_cast<std::string>(start_inside));
                
                score += score_inside;
                
                pre_edge_inside = edge_inside;
            }
            
            if ( i == 0 || best_score < score) {
                best_score = score;
                best_edge_id = edge_set[i];
                advance_outer = advance;
                start_outer2 = start;
                
                DebugUtility::Print(DebugUtility::Normal, "\tUpdate best edge... best score = " + boost::lexical_cast<std::string>(score) + ", best edge = " + boost::lexical_cast<std::string>(best_edge_id) + ", start = " + boost::lexical_cast<std::string>(start_outer));
            }
            
        } // for (int i = 0; i < edge_set.size(); ++i)
        
        if (best_edge_id == -1) {
            has_error_ = true;
            return matched;
        }
        start_outer = start_outer2;
        
        DebugUtility::Print(DebugUtility::Normal, "Best edge is [id, x1, y1, x2, y2]: [" +
                            boost::lexical_cast<std::string>(best_edge_id) + "," +
                            boost::lexical_cast<std::string>(tree_->GetEdge(best_edge_id).at(0).get<0>()) + ", " +
                            boost::lexical_cast<std::string>(tree_->GetEdge(best_edge_id).at(0).get<1>()) + ", " +
                            boost::lexical_cast<std::string>(tree_->GetEdge(best_edge_id).at(1).get<0>()) + ", " +
                            boost::lexical_cast<std::string>(tree_->GetEdge(best_edge_id).at(1).get<1>()) + "]");
        
        DebugUtility::Print(DebugUtility::Normal, "\tBest score = " + boost::lexical_cast<std::string>(best_score));
        
        this->UpdatePointAndCandidateEdge(advance_outer, start_outer, pre_edge_outer, edge_set, current_gps);
        
        DebugUtility::Print(DebugUtility::Normal, "Update GPS point and Candidate edge set... current gps = " + boost::lexical_cast<std::string>(current_gps) + ", edge set size = " + boost::lexical_cast<std::string>(edge_set.size()) + ", start vertex: " + boost::lexical_cast<std::string>(start_outer) + ", advance outer: " +
                            boost::lexical_cast<std::string>(advance_outer));
       
        if (matched.size() == 0 || matched[matched.size() - 1] != best_edge_id) {
            matched.push_back(best_edge_id);
        }
        
        pre_edge_outer = best_edge_id;
    }
    
    return matched;
}

bool MMDensity::MatchPoint2Edge(int point_id, int edge, int pre_edge, double& score, bool& advance, int& start) {
   // DebugUtility::Print(DebugUtility::Verbose, "start = " + boost::lexical_cast<std::string>(start));
    
    // score function
    // s = sd + sa
    // sd(pi, ci) = ud - a * d(pi, ci)^nd
    // sa(pi, ci) = ua * cos(ai,j)^na
    double a = 0.17;
    double nd = 1.4;
    double ud = a * pow(dist_parameter_, 1.4); // 10
    
    double ua = ud; // 10
    double na = 3;  // 必须是奇数
    
    // GPS 点
    double x, y, pre_x, pre_y;
    GPSPoint point = tree_->GetTrajectory(match_id_)[point_id];
    GPSPoint pre_point;
    if (point_id != 0) {
        pre_point = tree_->GetTrajectory(match_id_)[point_id - 1];
    } else {
        pre_point = tree_->GetTrajectory(match_id_)[1];
    }
    x = point.x_; y = point.y_;
    pre_x = pre_point.x_; pre_y = pre_point.y_;
    
    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tGet GPS point detail (id, x, y) = (" +
                        boost::lexical_cast<std::string>(point_id) + ", " +
                        boost::lexical_cast<std::string>(x) + ", " +
                        boost::lexical_cast<std::string>(y) + ")");
    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tGet pre GPS point detail (x, y) = (" +
                        boost::lexical_cast<std::string>(pre_x) + ", " +
                        boost::lexical_cast<std::string>(pre_y) + ")");
    
    // 候选边
    BoostLineString line = tree_->GetEdge(edge);
    std::vector<int> node_index = tree_->GetEdgeNode(edge);
    double x1 = line.at(0).get<0>();
    double y1 = line.at(0).get<1>();
    double x2 = line.at(1).get<0>();
    double y2 = line.at(1).get<1>();
    
    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tGet edge detial [id, x1, y1, x2, y2]: [" +
                        boost::lexical_cast<std::string>(edge) + "," +
                        boost::lexical_cast<std::string>(x1) + ", " +
                        boost::lexical_cast<std::string>(y1) + ", " +
                        boost::lexical_cast<std::string>(x2) + ", " +
                        boost::lexical_cast<std::string>(y2) + "]");
    // 投影点
    double xx, yy;
    advance = GeometryUtility::GetProjectPoint(x1, y1, x2, y2, x, y, xx, yy);
    
    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tGet projection point detail (inside, x, y) = (" +
                        boost::lexical_cast<std::string>(advance) + ", " +
                        boost::lexical_cast<std::string>(xx) + ", " +
                        boost::lexical_cast<std::string>(yy) + ")");
    
    // 距离权重
    double sd = 0;
    double dist = sqrt((x - xx) * (x - xx) + (y - yy) * (y - yy));
    sd = ud - a * pow(dist, nd);
    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tDistance score = " + boost::lexical_cast<std::string>(sd));
    
    // 朝向权重
    double sa = 0;
    double cos_theta;
    double m, n;
    double u, v;
    if (point_id != 0) {
        std::vector<int> node_index = tree_->GetEdgeNode(edge);
        if (start == node_index[0]) {
            u = x2 - x1;
            v = y2 - y1;
        } else if (start == node_index[1]) {
            u = x1 - x2;
            v = y1 - y2;
        } else {
            DebugUtility::Print(DebugUtility::Error, "Inconsistent GPS start: " + boost::lexical_cast<std::string>(start));
            DebugUtility::Print(DebugUtility::Error, "edge : [id, start, end] = [" +
                                boost::lexical_cast<std::string>(edge) + "," +
                                boost::lexical_cast<std::string>(node_index[0]) + "," +
                                boost::lexical_cast<std::string>(node_index[1]) + "]");

            return false;
        }
        
        if (edge == pre_edge) {
            u = -u;
            v = -v;
        }
        
        m = x - pre_x;
        n = y - pre_y;
        
        cos_theta = (u * m + v * n) / (sqrt(u * u + v * v) * sqrt(m * m + n * n));
    } else {
        m = pre_x - x;
        n = pre_y - y;
        
        double u = x2 - x1;
        double v = y2 - y1;
        double cos_theta1 = (u * m + v * n) / (sqrt(u * u + v * v) * sqrt(m * m + n * n));
        
        u = -u;
        v = -v;
        double cos_theta2 = (u * m + v * n) / (sqrt(u * u + v * v) * sqrt(m * m + n * n));
        
        if (cos_theta1 > cos_theta2) {
            cos_theta = cos_theta1;
            start = node_index[1];
        } else {
            cos_theta = cos_theta2;
            start = node_index[0];
        }
    }
    
    sa = ua * pow(cos_theta, na);
    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tOritation score = " + boost::lexical_cast<std::string>(sa));
    
    // 距离权重 + 朝向权重
    score = sd + sa;
    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tScore sum = " + boost::lexical_cast<std::string>(score));
    
    // 更新起始点
    UpdateStart(start, edge, pre_edge);

    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tUpdate start: " + boost::lexical_cast<std::string>(start));
    
    return true;
}

bool MMDensity::MatchPoint2EdgeSet(int point_id, std::vector<int>& edges, int pre_edge, double& score, bool& advance, int& edge_id, int& start) {
    score = -10000000000;
    
    for (int i = 0; i < edges.size(); ++i) {
        bool advance_inside;
        double score_inside;
        int start_inside = start;
        if (!this->MatchPoint2Edge(point_id, edges[i], pre_edge, score_inside, advance_inside, start_inside)) {
            return false;
        }
        
        
        DebugUtility::Print(DebugUtility::Verbose, "\t\t\t Match GPS point " + boost::lexical_cast<std::string>(point_id) + " to edge "  + boost::lexical_cast<std::string>(edges[i]) + "... score = " + boost::lexical_cast<std::string>(score_inside) + ", start = " + boost::lexical_cast<std::string>(start_inside));
        
        if (i == 0 || score < score_inside) {
            advance = advance_inside;
            score = score_inside;
            edge_id = edges[i];
        }
    }
    
    // 更新起始点
    UpdateStart(start, edge_id, pre_edge);
    
    return true;
}

void MMDensity::UpdateStart(int& start, int edge_id, int pre_edge) {
    if (pre_edge == -1) {
        return;
    }
    
    if (pre_edge == edge_id) {
        return;
    }
    
    if (!tree_->HasEdgeNode(edge_id)) {
        return;
    }
    
    std::vector<int> node_index = tree_->GetEdgeNode(edge_id);
    
    if (start == node_index[0]) {
        start = node_index[1];
    } else if (start == node_index[1]) {
        start = node_index[0];
    } else {
        DebugUtility::Print(DebugUtility::Error, "Error! In consistent!");
    }
}

void MMDensity::UpdatePointAndCandidateEdge(bool inside, int start, int pre_edge, std::vector<int>& edge_set, int& point_id) {
    edge_set.clear();
    
    if (!shapefile_graph_->HasEdgeOnVertex(start)) {
        DebugUtility::Print(DebugUtility::Error, "\t\t\tNo edge existing from edge!");
        return;
    }
    
    std::vector<int> full_edge_set = shapefile_graph_->GetEdgeOnVertex(start);
    
    if (inside) {
        point_id++;
        edge_set = full_edge_set;
    } else {
        for (int i = 0; i < full_edge_set.size(); ++i) {
            if (full_edge_set[i] == pre_edge)
                continue;
            
            edge_set.push_back(full_edge_set[i]);
        }
    }
    
    if (edge_set.size() == 0) {
        DebugUtility::Print(DebugUtility::Warning, "\t\t\tUpdate edge set fail, edge set size = 0, Original set size = " +
                            boost::lexical_cast<std::string>(full_edge_set.size()));
    }
    
    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tStart vertex " +
                        boost::lexical_cast<std::string>(start) + " has edge: " +
                        boost::lexical_cast<std::string>(edge_set.size()));
    DebugUtility::Print(DebugUtility::Verbose, "\t\t\tUpdate GPS point to " + boost::lexical_cast<std::string>(point_id));
    
}



