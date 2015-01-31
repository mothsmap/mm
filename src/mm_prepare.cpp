#include <iostream>
#include <fstream>

#include "debug.h"
#include "geometry.h"
#include "mm_route.h"
#include "mm_tree.h"
#include "mm_graph.h"
#include "mm_density_solver.h"

// boost
#include "boost/program_options/options_description.hpp"
#include "boost/program_options/variables_map.hpp"
#include "boost/program_options/cmdline.hpp"
#include "boost/program_options/parsers.hpp"
#include "boost/timer/timer.hpp"

std::string node;
std::string edge;
std::string history;
std::string out;

boost::shared_ptr<RTree> tree = boost::shared_ptr<RTree>(new RTree);
boost::shared_ptr<ShapefileGraph> graph = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
boost::shared_ptr<MMDensity> density_solver = boost::shared_ptr<MMDensity>(new MMDensity(tree, graph));

// 实验数据的范围
double xmin = 12701618.12;
double ymin = 4523858.73;
double xmax = 12796422.31;
double ymax = 4626505.94;
double expand = 1000.0;

bool ParseCommandLine (int argc, char** argv);

int main(int argc, char** argv) {
    if (!ParseCommandLine(argc, argv)) {
        std::cout << "Parse command line arguments fail!";
        return -1;
    }
    
    {
        std::cout << "\nAdd Trajectory logs...\n";
        boost::timer::auto_cpu_timer t;
        
        if (!tree->BuildTrajectory(history, xmin - expand, ymin - expand, xmax + expand, ymax + expand)) {
            std::cout << "Add Trajectory logs fail!\n";
            
            return false;
        }
    }
    
    {
        std::cout << "\nBuild RTree for road network...\n";
        boost::timer::auto_cpu_timer t;
        // 加载路网、结点、历史轨迹数据，以树的形式组织
        if (!tree->BuildRoad(node, edge, xmin - expand, ymin - expand, xmax + expand, ymax + expand)) {
            std::cout << "Build RTree for road network!\n";
            
            return false;
        }
    }
    
    {
        std::cout << "\nBuild Graph for road network...\n";
        boost::timer::auto_cpu_timer t;
        // 建立图结构
        if (!graph->Build(xmin - expand, ymin - expand, xmax + expand, ymax + expand)) {
            std::cout << "Build Graph for road network!\n";
            
            return false;
        }
    }
    
    {
        std::cout << "\nMatch history trajectory...\n";
        boost::timer::auto_cpu_timer t;
        // 对于每一条历史轨迹进行匹配
        density_solver->Match();
    }
    
    {
        std::cout << "\nSave..\n";
        boost::timer::auto_cpu_timer t;
        graph->Save(out + "/graph.xml");
        tree->SaveRoad(out + "/roads.xml");
        tree->SaveTrajectory(out + "/trajectory.xml");
    }
    
    return 0;
}

bool ParseCommandLine (int argc, char** argv) {
    boost::program_options::options_description options_desc ("Prepare Options");
    options_desc.add_options()
    ("help", "print help information.")
    
    ("edge", boost::program_options::value<std::string>(), "edge shapefile")
    ("node", boost::program_options::value<std::string>(), "node shapefile")
    ("history", boost::program_options::value<std::string>(), "history shapefile")
    ("out", boost::program_options::value<std::string>(), "out directory")
    ;
    
    boost::program_options::variables_map variables_map;
    
    try {
        boost::program_options::store (boost::program_options::parse_command_line (argc, argv, options_desc), variables_map);
        boost::program_options::notify (variables_map);
        
        if (variables_map.size() == 0 || variables_map.count ("help")) {
            std::cout << options_desc << std::endl;
            return false;
        }
        
        if (variables_map.count ("edge")) {
            edge = variables_map["edge"].as<std::string>();
        } else {
            std::cout << "\edge file is not specified!\n";
            std::cout << options_desc << std::endl;
            return false;
        }
        
        if (variables_map.count ("node")) {
            node = variables_map["node"].as<std::string>();
        } else {
            std::cout << "\node file is not specified!\n";
            std::cout << options_desc << std::endl;
            return false;
        }
        
        if (variables_map.count ("history")) {
            history = variables_map["history"].as<std::string>();
        } else {
            std::cout << "\nhistory file is not specified!\n";
            std::cout << options_desc << std::endl;
            return false;
        }
        
        if (variables_map.count ("out")) {
            out = variables_map["out"].as<std::string>();
        } else {
            std::cout << "\nout file is not specified!\n";
            std::cout << options_desc << std::endl;
            return false;
        }
        
        if (variables_map.count ("xmin")) {
            xmin = variables_map["xmin"].as<double>();
        }
        
        if (variables_map.count ("xmax")) {
            xmax = variables_map["xmax"].as<double>();
            
        }
        
        if (variables_map.count ("ymin")) {
            ymin = variables_map["ymin"].as<double>();
            
        }
        
        if (variables_map.count ("ymax")) {
            ymax = variables_map["ymax"].as<double>();
            
        }
    } catch (boost::program_options::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << options_desc << std::endl;
        return false;
    }
    
    return true;
}