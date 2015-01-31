#include <iostream>
#include <fstream>

#include "debug.h"
#include "geometry.h"
#include "mm_route.h"
#include "mm_tree.h"
#include "mm_graph.h"
#include "mm_density_solver.h"

#include <boost/timer/timer.hpp>

using namespace std;

// 数据目录存放着路网（edges.shp),结点(nodes.shp)和历史轨迹数据(history.shp)
std::string raw_dir = "/Volumes/second/mm_data/raw";
std::string out_dir = "/Volumes/second/mm_data/prepare";
std::string node = raw_dir + "/nodes.shp";
std::string edge = raw_dir + "/edges.shp";
std::string history = raw_dir + "/history.shp";

boost::shared_ptr<RTree> tree = boost::shared_ptr<RTree>(new RTree);
boost::shared_ptr<ShapefileGraph> graph = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
boost::shared_ptr<MMDensity> density_solver = boost::shared_ptr<MMDensity>(new MMDensity(tree, graph));

// 实验数据的范围
const double xmin = 12701618.12;
const double ymin = 4523858.73;
const double xmax = 12796422.31;
const double ymax = 4626505.94;
const double expand = 1000.0;

//const double xmin = 12738480.50;
//const double ymin = 4587841.67;
//const double xmax = 12741801.68;
//const double ymax = 4588509.55;
//const double expand = 1000.0;

//#define TEST

int main() {
#ifdef TEST
    
    graph->Load(data_dir + "/graph.xml");
    
    tree->LoadRoad(data_dir + "/roads.xml");
    
    tree->LoadTrajectory(data_dir + "/trajectory.xml");
#else
    {
        std::cout << "\nAdd Trajectory logs...\n";
        boost::timer::auto_cpu_timer t;
        
        if (!tree->BuildTrajectory(history, xmin - expand, ymin - expand, xmax + expand, ymax + expand)) {
            std::cout << "Add Trajectory logs!\n";
            
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
        graph->Save(out_dir + "/graph.xml");
        tree->SaveRoad(out_dir + "/roads.xml");
        tree->SaveTrajectory(out_dir + "/trajectory.xml");
    }
    
#endif
    
    //    // 保存成GeoJson的格式
    //    std::string filename = data_dir + "/" + boost::lexical_cast<std::string>(0) + ".geojson";
    //    std::string json = tree->GetMatchedTrajectoryAsGeoJson(0);
    //    std::ofstream ofs(filename);
    //    ofs << json << std::endl;
    //    ofs.close();
    
    return 0;
}