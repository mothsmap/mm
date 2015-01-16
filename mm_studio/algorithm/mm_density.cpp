#include "mm_density.h"
#include "rtree.h"
#include "route.h"
#include "shapefile_graph.h"

MMDensity::MMDensity(boost::shared_ptr<RTree> rtree,
                     boost::shared_ptr<Route> route,
                     boost::shared_ptr<ShapefileGraph> shapefile_graph) {
    tree_ = rtree;
    route_ = route;
    shapefile_graph_ = shapefile_graph;
}

MMDensity::~MMDensity() {
    
}

std::vector<int>& MMDensity::Match() {
    // GPS 点序列
    const std::vector<wxPoint2DDouble>& points = route_->getRoute();
    std::cout << "Match " << points.size() << " GPS points...\n";
    
    // 当前gps点
    int current_gps = 0;
    
    // 当前GPS点邻近的边集， 100米内的邻近边
    int step_size = 100;
    std::vector<Value> edge_set = tree_->Query(EDGE, points[0].m_x - step_size, points[0].m_y - step_size, points[0].m_x + step_size, points[0].m_y + step_size);
    while (edge_set.size() <= 0) {
        step_size += 25;
        edge_set = tree_->Query(EDGE, points[0].m_x - step_size, points[0].m_y - step_size, points[0].m_x + step_size, points[0].m_y + step_size);
    }
    
    while (current_gps < points.size()) {
        std::cout << "GPS #" << current_gps << " has " << edge_set.size() << "edges.\n";
        
        // 当前GPS点匹配的最佳得分
        double best_score = -10000000000.0;
        // 最佳匹配边
        int best_edge_id = -1;
        // 是否要前往下个GPS点
        bool advance_outer;
        
        // 当前边的起始点
        double start_x, start_y;
        
        // 循环处理当前GPS点的候选边集，寻找最佳边
        for (int i = 0; i < edge_set.size(); ++i) {
            // 首先得到将该GPS点匹配到当前边
            bool advance;
            double score;
            this->MatchPoint2Edge(current_gps, edge_set[i], score, advance, start_x, start_y);
            
            // 往前探索3步
            Value e0 = edge_set[i];
            bool advance_inside = advance;
            int current_gps_inside = current_gps;
            double start_inside_x = start_x;
            double start_inside_y = start_y;
            for (int i = 1; i <= 3; ++i) {
                if (current_gps_inside >= edge_set.size())
                    break;
                    
                double score_inside;
                std::vector<Value> edge_set_inside;
                int edge_inside;
                this->UpdatePointAndCandidateEdge(advance_inside, e0, start_inside_x, start_inside_y, edge_set_inside, current_gps_inside);
                
                this->MatchPoint2EdgeSet(current_gps_inside, edge_set_inside, score_inside, advance_inside, edge_inside, start_inside_x, start_inside_y);
                
                score += score_inside;
                e0 = edge_set_inside[edge_inside];
            }
            
            if (best_score < score) {
                best_score = score;
                best_edge_id = i;
                advance_outer = advance;
            }
        }
        Value best_edge = edge_set[best_edge_id];
        this->UpdatePointAndCandidateEdge(advance_outer, best_edge, start_x, start_y, edge_set, current_gps);
        
        match_.push_back(best_edge.second);
    }
    return match_;
}

void MMDensity::MatchPoint2Edge(int point_id, Value& edge, double& score, bool& advance, double& start_x, double& start_y) {
    // score function
    // s = sd + sa
    // sd(pi, ci) = ud - a * d(pi, ci)^nd
    // sa(pi, ci) = ua * cos(ai,j)^na
    double ud = 10;
    double a = 0.17;
    double nd = 1.4;
    
    double ua = 10;
    double na = 4;
    
    // GPS 点
    double x, y, pre_x, pre_y;
    route_->getRouteVertex(point_id, x, y);
    if (point_id != 0) {
        route_->getRouteVertex(point_id - 1, pre_x, pre_y);
    } else {
        route_->getRouteVertex(1, pre_x, pre_y);
    }
    
    // 候选边
    int geometry_id = edge.second;
    BoostLineString line = tree_->GetEdge(geometry_id);
    double x1 = line.at(0).get<0>();
    double y1 = line.at(0).get<1>();
    double x2 = line.at(1).get<0>();
    double y2 = line.at(1).get<1>();
    
    // 投影点
    double xx, yy;
    advance = GetProjectPoint(x1, y1, x2, y2, x, y, xx, yy);
    
    // 距离权重
    double sd = 0;
    double dist = sqrt((x - xx) * (x - xx) + (y - yy) * (y - yy));
    sd = ud - a * pow(dist, nd);
    
    // 朝向权重
    double sa = 0;
    double cos_theta;
    double m, n;
    double u, v;
    if (point_id != 0) {
        if (PointEqual(x1, y1, start_x, start_y)) {
            u = x2 - x1;
            v = y2 - y1;
        } else if (PointEqual(x2, y2, start_x, start_y)) {
            u = x1 - x2;
            v = y1 - y2;
        } else {
            std::cout << "Error! Inconsistent!\n";
            u = v = 0;
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
            start_x = x2;
            start_y = y2;
        } else {
            cos_theta = cos_theta2;
            start_x = x1;
            start_y = y1;
        }
    }
    
    sa = ua * pow(cos_theta, na);
    
    // 距离权重 + 朝向权重
    score = sd + sa;
    
    // 更新起始点
    UpdateStart(start_x, start_y, edge.second);
}

void MMDensity::MatchPoint2EdgeSet(int point_id, std::vector<Value>& edges, double& score, bool& advance, int& edge_id, double& start_x, double& start_y) {
    score = -10000000000;
    for (int i = 0; i < edges.size(); ++i) {
        bool advance_inside;
        double score_inside;
        this->MatchPoint2Edge(point_id, edges[i], score_inside, advance_inside, start_x, start_y);
        
        if (score < score_inside) {
            advance = advance_inside;
            score = score_inside;
            edge_id = i;
        }
    }
    
    // 更新起始点
    UpdateStart(start_x, start_y, edge_id);
}

void MMDensity::UpdateStart(double& start_x, double& start_y, int edge_id) {
    BoostLineString line = tree_->GetEdge(edge_id);
    double x1 = line.at(0).get<0>();
    double y1 = line.at(0).get<1>();
    double x2 = line.at(1).get<0>();
    double y2 = line.at(1).get<1>();
    if (PointEqual(x1, y1, start_x, start_y)) {
        start_x = x2;
        start_y = y2;
    } else if (PointEqual(x2, y2, start_x, start_y)) {
        start_x = x1;
        start_y = y1;
    } else {
        std::cout << "Error! In consistent!\n";
    }
}

void MMDensity::UpdatePointAndCandidateEdge(bool advance, Value& edge, double start_x, double start_y, std::vector<Value>& edge_set, int& point_id) {
    edge_set.clear();
    
    const std::vector<wxPoint2DDouble>& points = route_->getRoute();
    int step_size = 800;
    std::vector<Value> large_set = tree_->Query(EDGE, points[point_id].m_x - step_size, points[point_id].m_y - step_size, points[point_id].m_x + step_size, points[point_id].m_y + step_size);
    if (large_set.size() <= 0)
        std::cout << "large set is empty!\n";
    
    for (int i = 0; i < large_set.size(); ++i) {
        BoostLineString line1 = tree_->GetEdge(edge.second);
        BoostLineString line = tree_->GetEdge(large_set[i].second);
        
        if (!advance && LineEqual(line1, line)) {
            continue;
        }
        
        double x1 = line.at(0).get<0>();
        double y1 = line.at(0).get<1>();
        double x2 = line.at(1).get<0>();
        double y2 = line.at(1).get<1>();
        
        if (PointEqual(x1, y1, start_x, start_y) || PointEqual(x2, y2, start_x, start_y)) {
            std::cout << "Add a candidate edge!\n";
            edge_set.push_back(large_set[i]);
        }
    }
    
    if (edge_set.size() == 0) {
        std::cout << "Error! Edge set is empty!\n";
    }
    
    // 更新当前GPS点
    if (advance) {
        point_id++;
    }
}

bool MMDensity::GetProjectPoint(double x1, double y1, double x2, double y2, double x, double y, double& xx, double& yy) {
    // 两个端点在x-y方向的最大最小值
    double xmin = x1 < x2 ? x1 : x2;
    double xmax = x1 > x2 ? x1 : x2;
    double ymin = y1 < y2 ? y1 : y2;
    double ymax = y1 > y2 ? y1 : y2;
    
    // x最小值对应的y
    double xmin2y = x1 < x2 ? y1 : y2;
    // x最大值对应的y
    double xmax2y = x1 > x2 ? y1 : y2;
    
    // 直线参数 y = k * x + b
    double k, b;
    
    bool inside = true;
    // 如果线段平行于X轴
    if (abs(y2 - y1) < VERY_SMALL_NUMBER) {
        if (x <= xmin) {
            xx = xmin;
            inside = false;
        } else if (x >= xmax) {
            inside = false;
            xx = xmax;
        } else {
            inside = true;
            xx = x;
        }
        yy = y1;
        
    } else if (abs(x2 - x1) < VERY_SMALL_NUMBER) {
        // 如果线段平行于y轴
        if (y <= ymin) {
            inside = false;
            yy = ymin;
        } else if (y >= ymax) {
            inside = false;
            yy = ymax;
        } else {
            inside = true;
            yy = y;
        }
        xx = x1;
    } else {
        // 如果线段不平行于x、y轴
        k = (y2 - y1) / (x2 - x1);
        b = y2 - k * x2;
        
        xx = (k * y + x - k * b) / (k * k + 1);
        yy = k * xx + b;
        
        if (xx <= xmin) {
            inside = false;
            xx = xmin;
            yy = xmin2y;
        } else if (xx >= xmax) {
            inside = false;
            xx = xmax;
            yy = xmax2y;
        } else {
            inside = true;
        }
    }
    
    return inside;
}

bool MMDensity::PointEqual(double x1, double y1, double x2, double y2) {
    return (fabs(x1 - x2) <= VERY_SMALL_NUMBER && fabs(y1 - y2) <= VERY_SMALL_NUMBER);
}

bool MMDensity::LineEqual(BoostLineString& line1, BoostLineString& line2) {
    double x1 = line1.at(0).get<0>();
    double y1 = line1.at(0).get<1>();
    double x2 = line1.at(1).get<0>();
    double y2 = line1.at(1).get<1>();
    
    double m1 = line2.at(0).get<0>();
    double n1 = line2.at(0).get<1>();
    double m2 = line2.at(1).get<0>();
    double n2 = line2.at(1).get<1>();
    
    if (PointEqual(x1, y1, m1, n1) && PointEqual(x2, y2, m2, n2)) {
        return true;
    }
    
    if (PointEqual(x1, y1, m2, n2) && PointEqual(x2, y2, m1, n1)) {
        return true;
    }
    
    return false;
}

