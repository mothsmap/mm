#include <iostream>
#include <fstream>

#include "debug.h"
#include "geometry.h"
#include "mm_route.h"
#include "mm_tree.h"
#include "mm_graph.h"
#include "mm_density_solver.h"

using namespace std;

// 数据目录存放着路网（edges.shp),结点(nodes.shp)和历史轨迹数据(history.shp)
std::string data_dir = "/Volumes/second/mm_data_test";
std::string node = data_dir + "/nodes.shp";
std::string edge = data_dir + "/edges.shp";
std::string history = data_dir + "/history.shp";

boost::shared_ptr<RTree> tree = boost::shared_ptr<RTree>(new RTree);
boost::shared_ptr<ShapefileGraph> graph = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph(tree));
boost::shared_ptr<MMDensity> density_solver = boost::shared_ptr<MMDensity>(new MMDensity(tree, graph));

// 实验数据的范围
#if 0
const double xmin = 12701618.12;
const double ymin = 4523858.73;
const double xmax = 12796422.31;
const double ymax = 4626505.94;
const double expand = 1000.0;
#endif

const double xmin = 12738480.50;
const double ymin = 4587841.67;
const double xmax = 12741801.68;
const double ymax = 4588509.55;
const double expand = 1000.0;

int main() {
    // 加载路网、结点、历史轨迹数据，以树的形式组织
    if (!tree->Build(node, edge, history, xmin - expand, ymin - expand, xmax + expand, ymax + expand)) {
        std::cout << "Build RTree fail!\n";
        
        return false;
    }
    
    // 建立图结构
    if (!graph->Build(xmin - expand, ymin - expand, xmax + expand, ymax + expand)) {
        std::cout << "Build Graph fail!\n";
        
        return false;
    }
    
    for (int i = 0; i < tree->trajectory_size(); ++i) {
        // 对于每一条历史轨迹进行匹配
        std::vector<int> matched_route = density_solver->Match(i);
        
        tree->InsertMatchedTrajectory(matched_route, i);
        
        // 保存成GeoJson的格式
        std::string filename = data_dir + "/" + boost::lexical_cast<std::string>(i) + ".geojson";
        std::string json = tree->GetMatchedTrajectoryAsGeoJson(i);
        std::ofstream ofs(filename);
        ofs << json << std::endl;
        ofs.close();
    }
    
    graph->Save(data_dir);
    
    return 0;
}