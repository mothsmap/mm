#include "mm_route.h"
#include "mm_tree.h"
#include "geometry.h"
#include "debug.h"
#include "ogrsf_frmts.h"

Route::Route(void) {
}

Route::~Route(void) {
}

void Route::Reset() {
    route_.clear();
}

bool Route::Load(std::string filename) {
    route_.clear();
    
    OGRRegisterAll();
    OGRDataSource* data_source = OGRSFDriverRegistrar::Open(filename.c_str(), FALSE);
    if (data_source == NULL) {
        return false;
    }
    
    OGRLayer* layer = data_source->GetLayer(0);
    OGRFeature* feature = layer->GetNextFeature();
    while (feature != NULL) {
        // feature geometry
        OGRGeometry *poGeometry;
        poGeometry = feature->GetGeometryRef();
        OGRPoint *poPoint = (OGRPoint*) poGeometry;
        
        GPSPoint gps_point = {
            poPoint->getX(), poPoint->getY(),
            feature->GetFieldAsInteger("gpstime"),
            feature->GetFieldAsInteger("head"),
            feature->GetFieldAsInteger("speed")
        };
        
        route_.push_back(gps_point);
        
        OGRFeature::DestroyFeature(feature);
        feature = layer->GetNextFeature();
    }
    
#if 1
    std::cout << "Route: ";
    for (int i = 0; i < route_.size(); ++i) {
        std::cout << "( " << route_[i].x_ << ", " << route_[i].y_ << " )" << "\t";
    }
    std::cout << std::endl;
#endif
    
    candidate_sets_.resize(route_.size());
    
    return true;
}

void Route::InsertNode(double x, double y, int gps_time, int head, int speed) {
    GPSPoint gps_point = {
        x, y, gps_time, head, speed
    };
    
    route_.push_back(gps_point);
}

void Route::InsertCandidatePoint(CandidatePoint candidate_point) {
    candidate_sets_[candidate_point.gps_point_id].push_back(candidate_point);
}

void Route::Resample(double res) {
    GPSTrajectory resampled_route_;
    resampled_route_.push_back(route_[0]);
    GPSPoint pt_pre = route_[0];
    
    std::cout << "before resample: " << route_.size() << std::endl;
    
    for (int i = 1; i < route_.size(); ++i) {
        GPSPoint pt = route_[i];
        double distance = sqrt((pt.x_ - pt_pre.x_) * (pt.x_ - pt_pre.x_) + (pt.y_ - pt_pre.y_) * (pt.y_ - pt_pre.y_));
        if (distance > res) {
            resampled_route_.push_back(pt);
            pt_pre = pt;
        }
    }
    
    // 更新route
    route_.clear();
    for (int i = 0; i < resampled_route_.size(); ++i) {
        route_.push_back(resampled_route_[i]);
    }
    
    candidate_sets_.resize(route_.size());
    std::cout << "after resample: " << route_.size() << std::endl;
}

GPSPoint Route::getRouteVertex(int index) {
    return route_[index];
}

std::vector<int> FindSameItem(std::vector<Value>& from, std::vector<Value>& to) {
    std::vector<int> same_item;
    for (int i = 0; i < from.size(); ++i) {
        for (int j = 0; j < to.size(); ++j) {
            if (from[i].second == to[j].second) {
                same_item.push_back(from[i].second);
            }
        }
    }
    return same_item;
}

std::vector<int> FilterTrajectory(std::vector<int>& item, boost::shared_ptr<RTree> tree, int from_edge_id, int to_edge_id) {
    std::vector<int> result;
    double best_score = -1000;
    
    for (int i = 0; i < item.size(); ++i) {
        std::vector<int>& traj = tree->GetMatchedTrajectory(item[i]);
#if 0
        for (int i = 0; i < traj.size(); ++i) {
            std::cout << traj[i] << ", ";
        }
        std::cout << std::endl;
        
        std::cout << "from edge id: " << from_edge_id << ", to edge id: " << to_edge_id << std::endl;
#endif
        
        std::vector<int> filter_traj;
        double score = 0;
        int from_index = -1;
        int to_index = -1;
        
        for (int j = 0; j < traj.size(); ++j) {
            if (traj[j] == from_edge_id) {
                from_index = j;
            }
            if (traj[j] == to_edge_id) {
                to_index = j;
            }
            
            // 历史轨迹是有方向的，from_index 必须要小于等于 to_index
            if (from_index != -1 && to_index != -1 && from_index <= to_index) {
                for (int j = from_index; j <= to_index; ++j) {
                    filter_traj.push_back(traj[j]);
                    score = score + tree->GetRoadInfo(traj[j]).travel_counts_;
                }
                
                score /= (to_index - from_index) + 1;
                
                if (best_score < score) {
                    best_score = score;
                    result = filter_traj;
                }
            } // if
        } // for
    } // for
    
    return result;
}
    
    
bool Route::ComputeCandidateTrajectorySet(boost::shared_ptr<RTree> tree, double dist_thd) {
    DebugUtility::Print(DebugUtility::Normal, "GPS point size: " + boost::lexical_cast<std::string>(candidate_sets_.size()));
    
    for (int i = 0; i < candidate_sets_.size() - 1; ++i) {
        bool find = false;
        for (int m = 0; m < candidate_sets_[i].size(); ++m) {
            for (int n = 0; n < candidate_sets_[i + 1].size(); ++n) {
                CandidatePoint from = candidate_sets_[i][m];
                CandidatePoint to = candidate_sets_[i + 1][n];
                std::vector<Value> from_traj_set = tree->Query(TRAJECTORY, from.proj_x_ - dist_thd, from.proj_y_ - dist_thd, from.proj_x_ + dist_thd, from.proj_y_ + dist_thd);
                
                std::vector<Value> to_traj_set = tree->Query(TRAJECTORY, to.proj_x_ - dist_thd, to.proj_y_ - dist_thd, to.proj_x_ + dist_thd, to.proj_y_ + dist_thd);
                
                if (from_traj_set.size() == 0 || to_traj_set.size() == 0)
                    continue;
                
                // 找出共同的结果
                std::vector<int> same_item = FindSameItem(from_traj_set, to_traj_set);
                if (same_item.size() == 0)
                    continue;
                
                // 提取候选点之间的轨迹
                std::vector<int> best_traj = FilterTrajectory(same_item, tree, from.edge_id_, to.edge_id_);
                if (best_traj.size() == 0)
                    continue;
                
                // 插入
                CandidateTrajectory candidate_trajectory = {
                  i, i + 1, m, n, best_traj
                };
        
                candidate_trajectories_.push_back(candidate_trajectory);
                find = true;
            }
        }
        if (!find) {
            DebugUtility::Print(DebugUtility::Error, "GPS point " + boost::lexical_cast<std::string>(i) + " has no path to next GPS point");
        }
    }

    return true;
}

bool Route::ComputeCandidatePointSet(boost::shared_ptr<RTree> tree, double dist_thd) {
    const GPSTrajectory& points = this->getRoute();
    for (int i = 0; i < points.size(); ++i) {
        // std::vector<Value> result = tree_->Query(EDGE, BoostPoint(points[i].x_, points[i].y_), elements);
        std::vector<Value> result = tree->Query(EDGE, points[i].x_ - dist_thd, points[i].y_ - dist_thd, points[i].x_ + dist_thd, points[i].y_ + dist_thd);
        
        if (result.size() == 0) {
            DebugUtility::Print(DebugUtility::Error, "Candidate point set is empty!");
            return false;
        }
        
        // 点i的候选点
        for (int j = 0; j < result.size(); ++j) {
            int geometry_id = result[j].second;
            BoostLineString line = tree->GetEdge(geometry_id);
            EdgeProperties info = tree->GetRoadInfo(geometry_id);
            if (info.id_ != geometry_id) {
                DebugUtility::Print(DebugUtility::Error, "Edge id insistent!");
            }
            
            // 线段两个端点
            double x1 = line.at(0).get<0>();
            double y1 = line.at(0).get<1>();
            double x2 = line.at(1).get<0>();
            double y2 = line.at(1).get<1>();
            double xx, yy;
            GeometryUtility::GetProjectPoint(x1, y1, x2, y2, points[i].x_, points[i].y_, xx, yy);
            
            CandidatePoint cp = {
                i, // int gps_point_id;
                geometry_id, // int edge_id_;
                xx, yy, // double proj_x_, proj_y_;
                GeometryUtility::Distance(xx, yy, points[i].x_, points[i].y_) // double distance_;
            };
            
            this->InsertCandidatePoint(cp);
        }
    }
    
    return true;
}