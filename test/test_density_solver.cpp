#include <iostream>
#include <fstream>

#include "debug.h"
#include "geometry.h"
#include "mm_route.h"
#include "mm_tree.h"
#include "mm_graph.h"
#include "mm_density_solver.h"

#define BOOST_TEST_MODULE rtree_graph test
#include <boost/test//unit_test.hpp>

void DoTest(boost::shared_ptr<RTree> tree, boost::shared_ptr<ShapefileGraph> graph) {
    // rtree
    std::vector<Value> result = tree->Query(NODE, 5, 5, 15, 15);
    BOOST_CHECK_EQUAL(result.size(), 1);
    BOOST_CHECK_EQUAL(tree->GetNodeInfo(result[0].second).id_, 5);
    
    result = tree->Query(EDGE, 5, 5, 15, 15);
    BOOST_CHECK_EQUAL(result.size(), 8);
    
    // graph
    BOOST_REQUIRE(graph->Build(-10, -10, 50, 50));
    BOOST_CHECK(graph->HasEdgeOnVertex(0));
    std::vector<int> edges_on_vertex = graph->GetEdgeOnVertex(0);
    BOOST_CHECK_EQUAL(edges_on_vertex.size(), 3);
    edges_on_vertex = graph->GetEdgeOnVertex(5);
    BOOST_CHECK_EQUAL(edges_on_vertex.size(), 8);
    
    std::vector<int> path;
    double dist;
    BOOST_CHECK(graph->ShortestPath(0, 5, path, dist));
    BOOST_CHECK_EQUAL(path.size(), 1);
    BOOST_CHECK_EQUAL(path[0], 17);
    BOOST_CHECK(fabs(dist - 10 * sqrt(2.0)) < 0.01);
    
    path.clear();
    BOOST_CHECK(graph->ShortestPath(0, 11, path, dist));
    BOOST_CHECK_EQUAL(path.size(), 3);
    BOOST_CHECK_EQUAL(path[0], 17);
    BOOST_CHECK_EQUAL(path[1], 18);
    BOOST_CHECK_EQUAL(path[2], 8);
    BOOST_CHECK(fabs(dist - 20 * sqrt(2.0) - 10) < 0.01);
}

void TestDensitySolver(boost::shared_ptr<RTree> tree, boost::shared_ptr<ShapefileGraph> graph) {
    boost::shared_ptr<Route> route = boost::shared_ptr<Route>(new Route);
    double pts[6][2] = {
        {5, 4}, {10, 8}, {14, 10}, {18, 12}, {22, 12}, {23, 16}
    };
    for (int i = 0; i < 6; ++i) {
        route->InsertNode(pts[i][0], pts[i][1], 0, 0, 0);
    }
    
    tree->InsertGPSTrajectory(route->getRoute());
    
    boost::shared_ptr<MMDensity> density_solver = boost::shared_ptr<MMDensity>(new MMDensity(tree, graph));
    density_solver->set_dist_parameter(5);
    std::vector<int> result = density_solver->Match(0);
    
    BOOST_CHECK_EQUAL(result.size(), 3);
    BOOST_CHECK_EQUAL(result[0], 17);
    BOOST_CHECK_EQUAL(result[1], 4);
    BOOST_CHECK_EQUAL(result[2], 14);
}

BOOST_AUTO_TEST_CASE(test_tree) {
    boost::shared_ptr<RTree> tree = boost::shared_ptr<RTree>(new RTree);
    boost::shared_ptr<ShapefileGraph> graph = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
    
    const int node_count = 12;
    const int edge_count = 21;
    double nodes[node_count][2] = {
        {0, 0}, {10, 0}, {20, 0}, {30, 0},
        {0, 10}, {10, 10}, {20, 10}, {30, 10},
        {0, 20}, {10, 20}, {20, 20}, {30, 20}
    };
    int edges[edge_count][2] = {
        {0, 1}, {1, 2}, {2, 3},
        {4, 5}, {5, 6}, {6, 7},
        {8, 9}, {9, 10}, {10, 11},
        {0, 4}, {4, 8},
        {1, 5}, {5, 9},
        {2, 6}, {6, 10},
        {3, 7}, {7, 11},
        
        {0, 5}, {5, 10},
        {2, 5}, {5, 8}
    };
    
    for (int i = 0; i < node_count; ++i) {
        VertexProperties vp = {
            i, i, 0, nodes[i][0], nodes[i][1]
        };
        
        tree->InsertNode(nodes[i][0], nodes[i][1], vp);
    }
    
    for (int i = 0; i < edge_count; ++i) {
        double* n1 = nodes[edges[i][0]];
        double* n2 = nodes[edges[i][1]];

        EdgeProperties ep = {
            i, GeometryUtility::Distance(n1[0], n1[1], n2[0], n2[1]), 0, 0
        };
        
        tree->InsertRoad(n1[0], n1[1], n2[0], n2[1], ep);
    }
    
    DoTest(tree, graph);
    
    tree->SaveRoad("/Users/xuxiang/mm/data/test/rtree.xml");
    graph->Save("/Users/xuxiang/mm/data/test/graph.xml");
    
    boost::shared_ptr<RTree> tree2 = boost::shared_ptr<RTree>(new RTree);
    boost::shared_ptr<ShapefileGraph> graph2 = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
    tree2->LoadRoad("/Users/xuxiang/mm/data/test/rtree.xml");
    graph2->Load("/Users/xuxiang/mm/data/test/graph.xml");
    
    DoTest(tree2, graph2);
    
    TestDensitySolver(tree2, graph2);
}

