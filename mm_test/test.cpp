#include "boost_geometry.h"
#include "mm_density.h"
#include "rtree.h"
#include "route.h"
#include "shapefile_graph.h"

boost::shared_ptr<RTree> tree = boost::shared_ptr<RTree>(new RTree);
boost::shared_ptr<ShapefileGraph> graph = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
boost::shared_ptr<Route> route = boost::shared_ptr<Route>(new Route);
boost::shared_ptr<MMDensity> mm_density_solver = boost::shared_ptr<MMDensity>(new MMDensity(tree, route, graph));


void test_rtree() {
    std::cout << "Test RTree...\n";
    VertexProperties info = {
        0, // int id_;
        0, // int gps_id_;
        -1, // int candidate_id_;
        0, 0// double location_x_, location_y_;
    };
    tree->InsertNode(0, 0, info);
    
    info.id_++;
    info.gps_id_++;
    info.location_x_ = 100; info.location_y_ = 0;
    tree->InsertNode(100, 0, info);
    
    info.id_++;
    info.gps_id_++;
    info.location_x_ = 100; info.location_y_ = 50;
    tree->InsertNode(100, 50, info);
    
    info.id_++;
    info.gps_id_++;
    info.location_x_ = 150; info.location_y_ = 50;
    tree->InsertNode(150, 50, info);
    
    info.id_++;
    info.gps_id_++;
    info.location_x_ = 150; info.location_y_ = 150;
    tree->InsertNode(150, 150, info);
    
    info.id_++;
    info.gps_id_++;
    info.location_x_ = 200; info.location_y_ = 0;
    tree->InsertNode(200, 0, info);
    
    EdgeProperties einfo = {
        0, // int id_;
        100, // double weight_;
        0, //int oneway_;
        0 // int travel_counts_
    };
    tree->InsertRoad(0, 0, 100, 0, einfo);
    
    einfo.id_++;
    einfo.weight_ = 50;
    tree->InsertRoad(100, 0, 100, 50, einfo);
    
    einfo.id_++;
    einfo.weight_ = sqrt(100.0 * 100.0 + 50.0 * 50.0);
    tree->InsertRoad(0, 0, 100, 50, einfo);
    
    einfo.id_++;
    einfo.weight_ = 50.0;
    tree->InsertRoad(100, 50, 150, 50, einfo);
    
    einfo.id_++;
    einfo.weight_ = 50;
    tree->InsertRoad(150, 50, 150, 150, einfo);
    
    einfo.id_++;
    einfo.weight_ = sqrt(50.0 * 50.0 + 50.0 * 50.0);
    tree->InsertRoad(150, 50, 200, 0, einfo);
    
    std::cout << "#edges: " << tree->edge_size() << std::endl;
    std::cout << "#nodes: " << tree->node_size() << std::endl;
    
    // query edge
    std::vector<Value> result = tree->Query(QueryType::EDGE, 0, 0, 100, 100);
    std::cout << "query edge[0, 0, 100, 100]: \n";
    for (int i = 0; i < result.size(); ++i) {
        tree->PrintLine(result[i].second);
    }
    
    std::cout << "query edge[(100, 50), 3(elements)]: \n";
    result = tree->Query(QueryType::EDGE, BoostPoint(100, 50), 3);
    for (int i = 0; i < result.size(); ++i) {
        tree->PrintLine(result[i].second);
    }
    
    // query node
    result = tree->Query(QueryType::NODE, 0, 0, 100, 100);
    std::cout << "query node [0, 0, 100, 100]: \n";
    for (int i = 0; i < result.size(); ++i) {
        tree->PrintPoint(result[i].second);
    }
    
    std::cout << "query node [(100, 50), 3(elements)]: \n";
    result = tree->Query(QueryType::NODE, BoostPoint(100, 50), 3);
    for (int i = 0; i < result.size(); ++i) {
        tree->PrintPoint(result[i].second);
    }
}

void test_graph() {
    std::cout << "Test graph ...\n";
    graph->Build(0, 0, 200, 200);
    graph->ComputeShortestPath();
    
    graph->PrintShortestPath();
    graph->PrintEdgesOnVertex();
    
}

void test_density_solver() {
    std::cout << "Test density solfer ...\n";
    route->InsertNode(50, 10);
    route->InsertNode(80, 10);
    route->InsertNode(120, 10);
    route->InsertNode(130, 40);
    route->InsertNode(140, 40);
    route->InsertNode(160, 80);

    std::vector<int> matched = mm_density_solver->Match();
    
    for (int i = 0; i < matched.size(); ++i) {
        std::cout << " --> " << matched[i] << std::endl;
    }
    std::cout << std::endl;
}

int main() {
    
    test_rtree();
    test_graph();
    test_density_solver();
    
    return 0;
}