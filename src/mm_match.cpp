#include <iostream>
#include <fstream>

#include "debug.h"
#include "geometry.h"
#include "mm_route.h"
#include "mm_tree.h"
#include "mm_graph.h"
#include "mm_sparse_solver.h"

using namespace std;

// 数据目录存放着路网（edges.shp),结点(nodes.shp)和历史轨迹数据(history.shp)
std::string prepare_dir = "/Volumes/second/mm_data/prepare";
std::string data_dir = "/Volumes/second/mm_data/sparse_resample";
std::string out_dir = "/Volumes/second/mm_data/sparse_result";

// 输入
std::string input = data_dir + "/tx_1m.shp";
//std::string input = data_dir + "/../mm_data_test/test0.shp";

// 输出
std::string output = out_dir + "/tx.geojson";

boost::shared_ptr<RTree> tree = boost::shared_ptr<RTree>(new RTree);
boost::shared_ptr<ShapefileGraph> graph = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
boost::shared_ptr<Route> route = boost::shared_ptr<Route>(new Route);
boost::shared_ptr<MM> sparse_solver = boost::shared_ptr<MM>(new MM());


int main() {
    graph->Load(prepare_dir + "/graph.xml");
    tree->LoadRoad(prepare_dir + "/roads.xml");
    tree->LoadTrajectory(prepare_dir + "/trajectory.xml");
    
    // 加载稀疏点
    if (!route->Load(input)) {
        std::cout << "Loading route fail!\n";
        return -1;
    }
    
    // 寻找候选点集
    if (!route->ComputeCandidatePointSet(tree, 100)) {
        std::cout << "Find Candidate point set fail!\n";
        return -1;
    }
    
    // 寻找候选边集
    if (!route->ComputeCandidateTrajectorySet(tree, graph, 1.0)) {
        std::cout << "Find Candidate trajectory set fail!\n";
        return -1;
    }
    
    // 强化学习
    if (!sparse_solver->InitModel(tree, route)) {
        std::cout << "Init Learning Model fail!\n";
        return -1;
    }
    
    double score = sparse_solver->RunParsingAlgorithm(20000, 10000, 0.1);
    std::cout << "Score = " << score << std::endl;
    
    // 保存结果
    sparse_solver->SaveLearningResultAsGeojson(output);
    
    return 0;
}
