#include "debug.h"
#include "mm_tree.h"
#include <fstream>
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

bool RTree::BuildRoad(std::string node_file, std::string edge_file, double xmin, double ymin, double xmax, double ymax) {
    return (AddNodes(node_file, xmin, ymin, xmax, ymax) && AddEdges(edge_file, xmin, ymin, xmax, ymax));
}

bool RTree::BuildTrajectory(std::string history_file, double xmin, double ymin, double xmax, double ymax) {
    return AddGPSLogs(history_file, xmin, ymin, xmax, ymax);
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
    
    OGRDataSource* data_source = OGRSFDriverRegistrar::Open(map_dir.c_str(), FALSE);
    if (data_source == NULL) {
        DebugUtility::Print(DebugUtility::Error, "Open " + map_dir + " fail!");
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
    
    DebugUtility::Print(DebugUtility::Normal, "Add " + boost::lexical_cast<std::string>(node_count_) + " nodes.");
    return true;
}

bool RTree::AddEdges(std::string map_dir, double xmin, double ymin, double xmax, double ymax) {
    DebugUtility::Print(DebugUtility::Normal, "Add edges to Rtree...");
    edges_.clear();
    edge_tree_.clear();

    OGRDataSource* data_source = OGRSFDriverRegistrar::Open(map_dir.c_str(), FALSE);
    if (data_source == NULL) {
        DebugUtility::Print(DebugUtility::Error, "Open " + map_dir + " fail!");
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
    
    DebugUtility::Print(DebugUtility::Normal, "Add " + boost::lexical_cast<std::string>(edge_count_) + " edges.");
    return true;
}

bool RTree::AddGPSLogs(std::string map_dir, double xmin, double ymin, double xmax, double ymax) {
    DebugUtility::Print(DebugUtility::Normal, "Add GPS logs to Rtree...");
    
    // split logs to small trajectory...
    // In a trajectory, define:
    double near_dist;
    const double max_near_dist = 200;
    // 1> distance between near points should no bigger than 200m
    
    double near_time;
    const double max_near_time = 30;
    // 2> time slamp between near points should no bigger than 30ms
    
    double stay_time = 0;
    double max_stay_time = 60000;
    // 3> points with 0 speed should not continue 1min
    
    OGRDataSource* data_source = OGRSFDriverRegistrar::Open(map_dir.c_str(), FALSE);
    if (data_source == NULL) {
        DebugUtility::Print(DebugUtility::Error, "Open " + map_dir + " fail!");
        return false;
    }
    
    OGRLayer* layer = data_source->GetLayer(0);
    OGRFeature* feature = layer->GetNextFeature();
    GPSTrajectory trajectory;
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
        
        //int status = feature->GetFieldAsInteger("status");
        // Judge whether we should create a new trajectory
        if (trajectory.size() == 0) {
            trajectory.push_back(gps_point);
        } else {
            GPSPoint last_pt = trajectory[trajectory.size() - 1];
            near_dist = GeometryUtility::Distance(last_pt.x_, last_pt.y_, gps_point.x_, gps_point.y_);
            near_time = gps_point.t_ - last_pt.t_;
            if (last_pt.speed_ == 0 && gps_point.speed_ == 0) {
                stay_time += near_time;
            }
            
            if (near_dist > max_near_dist || near_time > max_near_time || stay_time > max_stay_time) {
                DebugUtility::Print(DebugUtility::Verbose, "Start a new trajectory with: near dist = " +
                                    boost::lexical_cast<std::string>(near_dist) +  " near time = " +
                                    boost::lexical_cast<std::string>(near_time) + " stay time = " +
                                    boost::lexical_cast<std::string>(stay_time));
                
                // start to record a new trajectory
                gps_trajectories_.insert(std::make_pair(trajectories_count_, trajectory));
                ++trajectories_count_;
                trajectory.clear();
                stay_time = 0;
            } else {
                trajectory.push_back(gps_point);
            }
        }
        
        OGRFeature::DestroyFeature (feature);
        feature = layer->GetNextFeature();
    }
    
    DebugUtility::Print(DebugUtility::Normal, "Add " + boost::lexical_cast<std::string>(trajectories_count_) + " trajectories.");
    
    return true;
}

void RTree::InsertMatchedTrajectory(std::vector<int> traj, int id) {
    matched_trajectories_.insert(std::make_pair(id, traj));
    
    double xmin, ymin, xmax, ymax;
    for (int i = 0; i < traj.size(); ++i) {
        // update edge polulaty
        this->TravelEdge(traj[i]);
        
        BoostLineString line = edges_.at(traj[i]);
        BoostBox b = bg::return_envelope<BoostBox>(line);
        
        if (i == 0) {
            xmin = b.min_corner().get<0>();
            ymin = b.min_corner().get<1>();
            xmax = b.max_corner().get<0>();
            ymax = b.max_corner().get<1>();
        } else {
            xmin = xmin > b.min_corner().get<0>() ? b.min_corner().get<0>() : xmin;
            ymin = ymin > b.min_corner().get<1>() ? b.min_corner().get<1>() : ymin;
            xmax = xmax < b.max_corner().get<0>() ? b.max_corner().get<0>() : xmax;
            ymax = ymax < b.max_corner().get<1>() ? b.max_corner().get<1>() : ymax;
        }
    }
    
    // update trajectory tree
    BoostBox bb(BoostPoint(xmin, ymin), BoostPoint(xmax, ymax));
    trajectory_tree_.insert(std::make_pair(bb, id));
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
    
 //   DebugUtility::Print(DebugUtility::Normal, "insert a road");
    
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

void RTree::SaveTrajectory(std::string filename) {
    std::ofstream ofs(filename);
    boost::archive::xml_oarchive oa(ofs);
    
    oa & BOOST_SERIALIZATION_NVP(trajectory_tree_);
    
    oa & BOOST_SERIALIZATION_NVP(gps_trajectories_);
    oa & BOOST_SERIALIZATION_NVP(matched_trajectories_);
    
    ofs.close();
}

void RTree::LoadTrajectory(std::string filename) {
    std::ifstream ifs(filename);
    
    boost::archive::xml_iarchive ia(ifs);
    ia & BOOST_SERIALIZATION_NVP(trajectory_tree_);
    
    ia & BOOST_SERIALIZATION_NVP(gps_trajectories_);
    ia & BOOST_SERIALIZATION_NVP(matched_trajectories_);
    
    ifs.close();
    
    trajectories_count_ = gps_trajectories_.size();
}

void RTree::SaveRoad(std::string filename) {
    std::ofstream ofs(filename);
    boost::archive::xml_oarchive oa(ofs);
    oa & BOOST_SERIALIZATION_NVP(node_tree_);
    oa & BOOST_SERIALIZATION_NVP(edge_tree_);
    
    oa & BOOST_SERIALIZATION_NVP(nodes_);
    oa & BOOST_SERIALIZATION_NVP(node_info_);
    
    oa & BOOST_SERIALIZATION_NVP(edges_);
    oa & BOOST_SERIALIZATION_NVP(edge_info_);
    
    oa & BOOST_SERIALIZATION_NVP(edge_node_map_);
    
    ofs.close();
}

void RTree::LoadRoad(std::string filename) {
    std::ifstream ifs(filename);
    boost::archive::xml_iarchive ia(ifs);
    ia & BOOST_SERIALIZATION_NVP(node_tree_);
    ia & BOOST_SERIALIZATION_NVP(edge_tree_);
    
    ia & BOOST_SERIALIZATION_NVP(nodes_);
    ia & BOOST_SERIALIZATION_NVP(node_info_);
    
    ia & BOOST_SERIALIZATION_NVP(edges_);
    ia & BOOST_SERIALIZATION_NVP(edge_info_);
    
    ia & BOOST_SERIALIZATION_NVP(edge_node_map_);
    
    ifs.close();
    
    node_count_ = nodes_.size();
    edge_count_ = edges_.size();
}

std::string RTree::GetMatchedTrajectoryAsGeoJson(int id) {
    std::string geojson = "{\"type\": \"FeatureCollection\",\"crs\": { \"type\": \"name\", \"properties\": { \"name\":\"urn:ogc:def:crs:EPSG::3857\" } },\"features\": [";
    
    for (int i = 0; i < matched_trajectories_[id].size(); ++i) {
        int geometry_id = matched_trajectories_[id][i];
        BoostLineString line_string = edges_.at(geometry_id);
        double x1 = line_string.at(0).get<0>();
        double y1 = line_string.at(0).get<1>();
        double x2 = line_string.at(1).get<0>();
        double y2 = line_string.at(1).get<1>();
        
        geojson += "{\"type\": \"Feature\", \"geometry\": {\"type\": \"LineString\", \"coordinates\": [[" + boost::lexical_cast<std::string>(x1) + "," + boost::lexical_cast<std::string>(y1) + + "], [" + boost::lexical_cast<std::string>(x2) + "," + boost::lexical_cast<std::string>(y2) + + "]]}}";
        
        if (i != matched_trajectories_[id].size() - 1)
            geojson += ",";
    }
    geojson += "]}";
    return geojson;
}

double RTree::GetRoadLength(int id) {
    BoostLineString line_string = edges_.at(id);
    double x1 = line_string.at(0).get<0>();
    double y1 = line_string.at(0).get<1>();
    double x2 = line_string.at(1).get<0>();
    double y2 = line_string.at(1).get<1>();
    
    return GeometryUtility::Distance(x1, y1, x2, y2);
}
