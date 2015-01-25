#ifndef __mm_tree__hh__
#define __mm_tree__hh__

#include "geometry.h"

// 查询类型
enum QueryType {
    NODE,
    EDGE,
    TRAJECTORY
};

class RTree {
public:
    RTree();
    ~RTree();
    
    /************** 构建函数 *************************/
    // 给定路网的结点和边所在的目录，建立R树
    bool Build(std::string node_file, std::string edge_file, std::string history_file, double xmin, double ymin, double xmax, double ymax);
    
    // 插入结点
    void InsertNode(double x, double y, VertexProperties info);
    // 插入道路
    void InsertRoad(double x1, double y1, double x2, double y2, EdgeProperties info);
    // 插入轨迹点
    void InsertGPSTrajectory(GPSTrajectory trajectory);
    // 插入轨迹点匹配到的道路边
    void InsertMatchedTrajectory(std::vector<int> traj, int id);
    
    void Save(std::string filename);
    void Load(std::string filename);
    
    /************** 查询函数 *************************/
    // 根据范围查询
    std::vector<Value> Query(QueryType type, double xmin, double ymin, double xmax, double ymax);
    
    // 查询点pt相邻的elements个要素
    std::vector<Value> Query(QueryType type, BoostPoint pt, int elements);
    
    /************** 添加历史经验 **********************/
    void TravelEdge(int id);
    
    /************** 获取信息 *************************/
    inline int edge_size() { return edge_count_; }
    inline int node_size() { return node_count_; }
    inline int trajectory_size() { return trajectories_count_; }
    
    inline BoostLineString GetEdge(int id) { return edges_.at(id); }
    inline BoostPoint GetNode(int id) { return nodes_.at(id); }
    inline GPSTrajectory& GetTrajectory(int id) { return gps_trajectories_.at(id); }
    inline std::vector<int>& GetMatchedTrajectory(int id) { return matched_trajectories_.at(id); }
    inline int GetMatchedTrajectorySize() { return matched_trajectories_.size(); }
    std::string GetMatchedTrajectoryAsGeoJson(int id);
    
    inline EdgeProperties GetRoadInfo(int id) { return edge_info_.at(id); }
    inline VertexProperties GetNodeInfo(int id) { return node_info_.at(id); }
    inline bool HasRoad(int id) { return edges_.find(id) != edges_.end(); }
    inline bool HasNode(int id) { return nodes_.find(id) != nodes_.end(); }
    
    inline std::vector<int>& GetEdgeNode(int id) { return edge_node_map_.at(id); }
    
    void PrintLine(int id);
    void PrintPoint(int id);
    
    void Reset();
private:
    // Build函数的辅助函数
    bool AddNodes(std::string map_dir, double xmin, double ymin, double xmax, double ymax);
    bool AddEdges(std::string map_dir, double xmin, double ymin, double xmax, double ymax);
    bool AddGPSLogs(std::string map_dir, double xmin, double ymin, double xmax, double ymax);
    
private:
    Boost_RTree edge_tree_, node_tree_, trajectory_tree_;
    
    std::map<int, BoostLineString>  edges_;
    std::map<int, EdgeProperties> edge_info_;
    
    std::map<int, BoostPoint> nodes_;
    std::map<int, VertexProperties> node_info_;
    
    std::map<int, GPSTrajectory> gps_trajectories_;
    std::map<int, std::vector<int> > matched_trajectories_;
    
    std::map<int, std::vector<int> > edge_node_map_;
    
    int edge_count_;
    int node_count_;
    int trajectories_count_;
};

#endif
