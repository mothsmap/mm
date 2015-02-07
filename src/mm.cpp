#include "mm.h"
#include <iostream>
#include <fstream>

#include "debug.h"
#include "geometry.h"
#include "mm_route.h"
#include "mm_tree.h"
#include "mm_graph.h"
#include "mm_density_solver.h"
#include "mm_sparse_solver.h"

MMSolver::MMSolver() {
    
}

MMSolver::~MMSolver() {
    
}

bool MMSolver::Prepare(std::string node, std::string edge, std::string history, std::string out_dir,
                 double xmin, double ymin, double xmax, double ymax) {
        boost::shared_ptr<RTree> tree = boost::shared_ptr<RTree>(new RTree);
        boost::shared_ptr<ShapefileGraph> graph = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
        boost::shared_ptr<MMDensity> density_solver = boost::shared_ptr<MMDensity>(new MMDensity(tree, graph));
        
        if (!tree->BuildTrajectory(history, xmin, ymin, xmax, ymax)) {
            std::cout << "Add Trajectory logs fail!\n";
            
            return false;
        }
        
        // 加载路网、结点、历史轨迹数据，以树的形式组织
        if (!tree->BuildRoad(node, edge, xmin, ymin, xmax, ymax)) {
            std::cout << "Build RTree for road network!\n";
            
            return false;
        }
        
        // 建立图结构
        if (!graph->Build(xmin, ymin, xmax, ymax)) {
            std::cout << "Build Graph for road network!\n";
            
            return false;
        }
        
        // 对于每一条历史轨迹进行匹配
        density_solver->Match();
        
        graph->Save(out_dir + "/graph.xml");
        tree->SaveRoad(out_dir + "/roads.xml");
        tree->SaveTrajectory(out_dir + "/trajectory.xml");
        
        return true;
    }

    bool MMSolver::Match(std::string preprocess,
                   std::string in,
                   std::string out,
                   std::string ground_truth) {
        
        boost::shared_ptr<RTree> tree = boost::shared_ptr<RTree>(new RTree);
        boost::shared_ptr<ShapefileGraph> graph = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
        boost::shared_ptr<Route> route = boost::shared_ptr<Route>(new Route);
        boost::shared_ptr<MM> sparse_solver = boost::shared_ptr<MM>(new MM());
        
        graph->Load(preprocess + "/graph.xml");
        tree->LoadRoad(preprocess + "/roads.xml");
        tree->LoadTrajectory(preprocess + "/trajectory.xml");
        
        // 加载稀疏点
        if (!route->Load(in)) {
            std::cout << "Loading route fail!\n";
            return false;
        }
        
        
        // 以100米为范围寻找候选点
        if (!route->ComputeCandidatePointSet(tree, 100)) {
            std::cout << "Find Candidate point set fail!\n";
            return false;
        }
        
        // 以1米为范围寻找候选路径
        if (!route->ComputeCandidateTrajectorySet(tree, graph, 1.0)) {
            std::cout << "Find Candidate trajectory set fail!\n";
            return false;
        }
        
        if (!sparse_solver->InitModel(tree, route)) {
            std::cout << "Init Learning Model fail!\n";
            return false;
        }
        
        // 学习3万次，学习步长为0.1
        double score = sparse_solver->RunParsingAlgorithm(20000, 10000, 0.1);
        std::cout << "Score = " << score << std::endl;
        
        // 保存结果
        sparse_solver->SaveLearningResultAsGeojson(out);
        
        // 评价
        if (ground_truth != "") {
            std::vector<int> matched_edges = sparse_solver->get_learning_result();
            
            boost::shared_ptr<Route> ground_truth_route = boost::shared_ptr<Route>(new Route);
            if (!ground_truth_route->Load(ground_truth)) {
                std::cout << "Load ground truth fail!\n";
                return false;
            }
            ground_truth_route->Resample(10);
            
            tree->InsertGPSTrajectory(ground_truth_route->getRoute());
            int ground_truth_id = tree->trajectory_size() - 1;
            
            boost::shared_ptr<MMDensity> density_solver = boost::shared_ptr<MMDensity>(new MMDensity(tree, graph));
            std::vector<int> ground_truth_edges = density_solver->Match(ground_truth_id);
            
            double accurate_rate = tree->CalculateAccurateRate(ground_truth_edges, matched_edges);
            
            std::cout << "\n\nAccurate rate: " << accurate_rate << std::endl;
        }
        
        return true;
    }
