#include "debug.h"
#include "mm_tree.h"
#include "ogrsf_frmts.h"

#define DIST_THD 10

RTree::RTree() {
    node_count_ = 0;
    edge_count_ = 0;
    trajectories_count_ = 0;
    
    OGRRegisterAll();
}

RTree::~RTree() {
}

bool RTree::Build(std::string map_dir, double xmin, double ymin, double xmax, double ymax) {
    return (AddNodes(map_dir, xmin, ymin, xmax, ymax) && AddEdges(map_dir, xmin, ymin, xmax, ymax) && AddGPSLogs(map_dir, xmin, ymin, xmax, ymax) );
}

std::vector<Value> RTree::Query(QueryType type, double xmin, double ymin, double xmax, double ymax) {
    BoostBox query_box(BoostPoint(xmin, ymin), BoostPoint(xmax, ymax));
    std::vector<Value> results;
    switch (type) {
        case EDGE:
            edge_tree_.query(bgi::intersects(query_box), std::back_inserter(results));
            break;
        case NODE:
            node_tree_.query(bgi::intersects(query_box), std::back_inserter(results));
            break;
        case TRAJECTORY:
            trajectory_tree_.query(bgi::intersects(query_box), std::back_inserter(results));
        default:
            break;
    }
    
    return results;
}

std::vector<Value> RTree::Query(QueryType type, BoostPoint pt, int elements) {
    std::vector<Value> results;
    switch (type) {
        case EDGE:
            edge_tree_.query(bgi::nearest(pt, elements), std::back_inserter(results));
            break;
        case NODE:
            node_tree_.query(bgi::nearest(pt, elements), std::back_inserter(results));
            break;
        case TRAJECTORY:
            trajectory_tree_.query(bgi::nearest(pt, elements), std::back_inserter(results));
        default:
            break;
    }
    
    return results;
}

bool RTree::AddNodes(std::string map_dir, double xmin, double ymin, double xmax, double ymax) {
    DebugUtility::Print(DebugUtility::Normal, "Add nodes to Rtree...");
    nodes_.clear();
    node_tree_.clear();
    
    std::string file = map_dir + "/mm/nodes.shp";
    OGRDataSource* data_source = OGRSFDriverRegistrar::Open(file.c_str(), FALSE);
    if (data_source == NULL) {
        DebugUtility::Print(DebugUtility::Error, "Open " + file + " fail!");
        return false;
    }
    
    OGRLayer* layer = data_source->GetLayer(0);
    layer->SetSpatialFilterRect(xmin, ymin, xmax, ymax);
    layer->ResetReading();
    OGRFeature* feature = layer->GetNextFeature();
    while (feature != NULL) {
        // feature geometry
        OGRGeometry* poGeometry = feature->GetGeometryRef();
        if (poGeometry == NULL || wkbFlatten(poGeometry->getGeometryType()) != wkbPoint) {
            OGRFeature::DestroyFeature (feature);
            feature = layer->GetNextFeature();
            continue;
        }
        
        // feature
        OGRPoint *point = (OGRPoint*) poGeometry;
        VertexProperties info = {
            node_count_, // int id_;
            -1, //int gps_id_;
            -1, //int candidate_id_;
            point->getX(), point->getY()//double location_x_, location_y_;
        };
        
        this->InsertNode(point->getX(), point->getY(), info);
        
        OGRFeature::DestroyFeature (feature);
        feature = layer->GetNextFeature();
    }
    
    return true;
}

bool RTree::AddEdges(std::string map_dir, double xmin, double ymin, double xmax, double ymax) {
    DebugUtility::Print(DebugUtility::Normal, "Add edges to Rtree...");
    edges_.clear();
    edge_tree_.clear();
    
    std::string file = map_dir + "/mm/edges.shp";
    OGRDataSource* data_source = OGRSFDriverRegistrar::Open(file.c_str(), FALSE);
    if (data_source == NULL) {
        DebugUtility::Print(DebugUtility::Error, "Open " + file + " fail!");
        return false;
    }
    
    OGRLayer* layer = data_source->GetLayer(0);
    layer->SetSpatialFilterRect(xmin, ymin, xmax, ymax);
    layer->ResetReading();
    OGRFeature* feature = layer->GetNextFeature();
    while (feature != NULL) {
        // feature geometry
        OGRGeometry* poGeometry = feature->GetGeometryRef();
        if (poGeometry == NULL || wkbFlatten(poGeometry->getGeometryType()) != wkbLineString) {
            OGRFeature::DestroyFeature (feature);
            feature = layer->GetNextFeature();
            continue;
        }
        
        // road
        OGRLineString *line_string = (OGRLineString*) poGeometry;
        OGRPoint start, end;
        line_string->StartPoint(&start);
        line_string->EndPoint(&end);
        // info
        EdgeProperties info = {
            edge_count_, // int id_;
            GeometryUtility::Distance(start.getX(), start.getY(), end.getX(), end.getY()), // double weight_;
            feature->GetFieldAsInteger("oneway"), // int oneway_;
            0 // int travel_counts_;
            
        };
        
        this->InsertRoad(start.getX(), start.getY(), end.getX(), end.getY(), info);
       
        OGRFeature::DestroyFeature (feature);
        feature = layer->GetNextFeature();
    }
    
    return true;
}

bool RTree::AddGPSLogs(std::string map_dir, double xmin, double ymin, double xmax, double ymax) {
    DebugUtility::Print(DebugUtility::Normal, "Add GPS logs to Rtree...");
    
        gps_trajectories_.clear();
    
        std::string filename = map_dir + "/mm/logs.shp";
        OGRDataSource* data_source = OGRSFDriverRegistrar::Open(filename.c_str(), FALSE);
        if (data_source == NULL) {
            DebugUtility::Print(DebugUtility::Error, "Open " + filename + " fail!");
            return false;
        }
        
        OGRLayer* layer = data_source->GetLayer(0);
        OGRFeature* feature = layer->GetNextFeature();
        GPSTrajectory trajectory;
        while (feature != NULL) {
            // feature geometry
            OGRGeometry *poGeometry;
            poGeometry = feature->GetGeometryRef();
            
            int status = feature->GetFieldAsInteger("status");
            if (status == 33280) {
                OGRPoint *poPoint = (OGRPoint*) poGeometry;
                GPSPoint gps_point = {
                    poPoint->getX(), poPoint->getY(),
                    feature->GetFieldAsInteger("gpstime"),
                    feature->GetFieldAsInteger("head"),
                    feature->GetFieldAsInteger("speed")
                };
                
                if (trajectory.size() > 0) {
                    GPSPoint pre_pt = trajectory[trajectory.size() - 1];
                    double dist = GeometryUtility::Distance(pre_pt.x_, pre_pt.y_, gps_point.x_, gps_point.y_);
                    if (dist > DIST_THD) {
                        trajectory.push_back(gps_point);
                    }
                } else {
                    trajectory.push_back(gps_point);
                }
            } else {
                if (trajectory.size() > 0) {
                    this->InsertGPSTrajectory(trajectory);
#if 1
                    std::cout << "insert a gps trajectory: ";
                    for (int i = 0; i < trajectory.size(); ++i) {
                        std::cout << trajectory[i].t_ << "\t";
                    }
                    std::cout << std::endl;
#endif
                    trajectory.clear();
                }
            }
            
            OGRFeature::DestroyFeature (feature);
            feature = layer->GetNextFeature();
        }
        
        return true;
}

void RTree::InsertMatchedTrajectory(std::vector<int> traj, int id) {
    matched_trajectories_.insert(std::make_pair(id, traj));
    
    for (int i = 0; i < traj.size(); ++i) {
        this->TravelEdge(traj[i]);
    }
}

void RTree::InsertNode(double x, double y, VertexProperties info) {
    node_info_.insert(std::make_pair(node_count_, info));
    
    BoostPoint boost_point(x, y);
    nodes_.insert(std::make_pair(node_count_, boost_point));
    
    // RTree
    BoostBox b = bg::return_envelope<BoostBox>(boost_point);
    node_tree_.insert(std::make_pair(b, node_count_));
    
    ++node_count_;
}

void RTree::InsertRoad(double x1, double y1, double x2, double y2, EdgeProperties info) {
    // edge node map
    std::vector<int> node_index;
    node_index.resize(2);
    bool find_start = false;
    bool find_end = false;
    for (int i = 0; i < node_info_.size(); ++i) {
        if (GeometryUtility::PointEqual(x1, y1, node_info_[i].location_x_, node_info_[i].location_y_)) {
            node_index[0] = node_info_.at(i).id_;
            find_start = true;
        }
        
        if (GeometryUtility::PointEqual(x2, y2, node_info_[i].location_x_, node_info_[i].location_y_)) {
            node_index[1] = node_info_.at(i).id_;
            find_end = true;
        }
    }
    
    if (find_start && find_end) {
        edge_node_map_.insert(std::make_pair(edge_count_, node_index));
    } else {
        DebugUtility::Print(DebugUtility::Warning, "insert an invalid road!");
        return ;
    }
    
    DebugUtility::Print(DebugUtility::Normal, "insert a road");
    
    edge_info_.insert(std::make_pair(edge_count_, info));
    
    BoostLineString boost_line;
    boost_line.push_back(BoostPoint(x1, y1));
    boost_line.push_back(BoostPoint(x2, y2));
    edges_.insert(std::make_pair(edge_count_, boost_line));
    
    // RTree
    BoostBox b = bg::return_envelope<BoostBox>(boost_line);
    edge_tree_.insert(std::make_pair(b, edge_count_));
    
    edge_count_++;
}

void RTree::InsertGPSTrajectory(GPSTrajectory trajectory) {
    if (trajectory.size() <= 3)
        return;
    
    BoostLineString line;
    for (int i = 0; i < trajectory.size(); ++i) {
        line.push_back(BoostPoint(trajectory[i].x_, trajectory[i].y_));
    }
    
    BoostBox b = bg::return_envelope<BoostBox>(line);
    trajectory_tree_.insert(std::make_pair(b, trajectories_count_));
    
    gps_trajectories_.insert(std::make_pair(trajectories_count_, trajectory));
    
    trajectories_count_++;
}

void RTree::PrintLine(int id) {
    BoostLineString line = this->GetEdge(id);
    
    double x1 = line.at(0).get<0>();
    double y1 = line.at(0).get<1>();
    double x2 = line.at(1).get<0>();
    double y2 = line.at(1).get<1>();
    
    std::cout << "(x1, y1), (x2, y2) = (" << x1 << ", " << y1 << "), (" << x2 << ", " << y2 << ")\n";
}

void RTree::PrintPoint(int id) {
    BoostPoint point = this->GetNode(id);
    std::cout << "(x1, y1) = (" << point.get<0>() << ", " << point.get<1>() << ")\n";
}

void RTree::Reset() {
    edge_tree_.clear();
    node_tree_.clear();
    
    edges_.clear();
    edge_info_.clear();
    
    nodes_.clear();
    node_info_.clear();
    
    gps_trajectories_.clear();
    
    edge_node_map_.clear();
    
    edge_count_ = 0;
    node_count_ = 0;
    trajectories_count_ = 0;
}

void RTree::TravelEdge(int id) {
    if (edge_info_.find(id) != edge_info_.end()) {
        EdgeProperties info = edge_info_.at(id);
        info.travel_counts_++;
        
        edge_info_.erase(edge_info_.find(id));
        edge_info_.insert(std::make_pair(id, info));
        
    } else {
        DebugUtility::Print(DebugUtility::Error, "Try to set edge info for a not exist edge!");
    }
}