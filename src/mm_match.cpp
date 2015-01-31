#include "debug.h"
#include "geometry.h"
#include "mm_route.h"
#include "mm_tree.h"
#include "mm_graph.h"
#include "mm_sparse_solver.h"
#include "mm_density_solver.h"

#include "boost/program_options/options_description.hpp"
#include "boost/program_options/variables_map.hpp"
#include "boost/program_options/cmdline.hpp"
#include "boost/program_options/parsers.hpp"

#include <iostream>
#include <fstream>

// 数据目录存放着路网（edges.shp),结点(nodes.shp)和历史轨迹数据(history.shp)
std::string preprocess;

// 输入输出
std::string in;
std::string out;
std::string ground_truth;

boost::shared_ptr<RTree> tree = boost::shared_ptr<RTree>(new RTree);
boost::shared_ptr<ShapefileGraph> graph = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
boost::shared_ptr<Route> route = boost::shared_ptr<Route>(new Route);
boost::shared_ptr<MM> sparse_solver = boost::shared_ptr<MM>(new MM());

bool ParseCommandLine (int argc, char** argv);
void Match();

int main(int argc, char** argv) {
    if (!ParseCommandLine(argc, argv)) {
        std::cout << "Parse command line fail!\n";
        return -1;
    }
    
    try {
        Match();
    } catch (std::exception e) {
        std::cout << "An error: " + boost::lexical_cast<std::string>(e.what()) << std::endl;
    } catch (...) {
        std::cout << "An unknow error!\n";
        return -1;
    }
    
    return 0;
}

void Match() {
    graph->Load(preprocess + "/graph.xml");
    tree->LoadRoad(preprocess + "/roads.xml");
    tree->LoadTrajectory(preprocess + "/trajectory.xml");
    
    // 加载稀疏点
    if (!route->Load(in)) {
        std::cout << "Loading route fail!\n";
        return;
    }
    
    // 寻找候选点集
    {
        std::cout << "\nFind Candidate point set...\n";
        boost::timer::auto_cpu_timer t;
        
        // 以100米为范围寻找候选点
        if (!route->ComputeCandidatePointSet(tree, 100)) {
            std::cout << "Find Candidate point set fail!\n";
            return;
        }
    }
    
    // 寻找候选边集
    {
        std::cout << "\nFind Candidate trajectory set...\n";
        boost::timer::auto_cpu_timer t;
        
        // 以1米为范围寻找候选路径
        if (!route->ComputeCandidateTrajectorySet(tree, graph, 1.0)) {
            std::cout << "Find Candidate trajectory set fail!\n";
            return;
        }
    }
    
    // 强化学习
    {
        std::cout << "\nInit Learning Model...\n";
        boost::timer::auto_cpu_timer t;
        
        if (!sparse_solver->InitModel(tree, route)) {
            std::cout << "Init Learning Model fail!\n";
            return;
        }
    }
    
    {
        std::cout << "\nLearning ...\n";
        boost::timer::auto_cpu_timer t;
        
        // 学习3万次，学习步长为0.1
        double score = sparse_solver->RunParsingAlgorithm(20000, 10000, 0.1);
        std::cout << "Score = " << score << std::endl;
    }
    
    // 保存结果
    sparse_solver->SaveLearningResultAsGeojson(out);
    
    // 评价
    if (ground_truth != "") {
        std::vector<int> matched_edges = sparse_solver->get_learning_result();
        
        boost::shared_ptr<Route> ground_truth_route = boost::shared_ptr<Route>(new Route);
        if (!ground_truth_route->Load(ground_truth)) {
            std::cout << "Load ground truth fail!\n";
            return;
        }
        ground_truth_route->Resample(10);
        
        tree->InsertGPSTrajectory(ground_truth_route->getRoute());
        int ground_truth_id = tree->trajectory_size() - 1;
        
        boost::shared_ptr<MMDensity> density_solver = boost::shared_ptr<MMDensity>(new MMDensity(tree, graph));
        std::vector<int> ground_truth_edges = density_solver->Match(ground_truth_id);
        
        double accurate_rate = tree->CalculateAccurateRate(ground_truth_edges, matched_edges);
        
        std::cout << "\n\nAccurate rate: " << accurate_rate << std::endl;
    }
}

bool ParseCommandLine (int argc, char** argv) {
    boost::program_options::options_description options_desc ("Match Options");
    options_desc.add_options()
    ("help", "print help information.")
    
    ("preprocess", boost::program_options::value<std::string>(), "the output directory of mm_prepare executable")
    ("in", boost::program_options::value<std::string>(), "input GPS trajectory in shapefile format")
    ("ground_truth", boost::program_options::value<std::string>(), "input ground truth GPS trajectory in shapefile format")
    ("out", boost::program_options::value<std::string>(), "output marched trajectory in GeoJson format")
    ;
    
    boost::program_options::variables_map variables_map;
    
    try {
        boost::program_options::store (boost::program_options::parse_command_line (argc, argv, options_desc), variables_map);
        boost::program_options::notify (variables_map);
        
        if (variables_map.size() == 0 || variables_map.count ("help")) {
            std::cout << options_desc << std::endl;
            return false;
        }
        
        if (variables_map.count ("preprocess")) {
            preprocess = variables_map["preprocess"].as<std::string>();
            std::cout << "preprocess directory: " << preprocess << std::endl;
        } else {
            std::cout << "\npreprocess directory is not specified!\n";
            std::cout << options_desc << std::endl;
            return false;
        }
        
        if (variables_map.count ("in")) {
            in = variables_map["in"].as<std::string>();
            std::cout << "in file: " << in << std::endl;
        } else {
            std::cout << "\nin file is not specified!\n";
            std::cout << options_desc << std::endl;
            return false;
        }
        
        if (variables_map.count ("ground_truth")) {
            ground_truth = variables_map["ground_truth"].as<std::string>();
            std::cout << "ground_truth file: " << ground_truth << std::endl;
        } else {
            ground_truth = "";
            return false;
        }
        
        if (variables_map.count ("out")) {
            out = variables_map["out"].as<std::string>();
            std::cout << "out file: " << out << std::endl;
        } else {
            std::cout << "\nout file is not specified!\n";
            std::cout << options_desc << std::endl;
            return false;
        }
    } catch (boost::program_options::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << options_desc << std::endl;
        return false;
    }
    
    return true;
}
