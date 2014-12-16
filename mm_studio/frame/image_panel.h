#ifndef MAP_CLIENT_H_
#define MAP_CLIENT_H_
#include "wx/wx.h"
#include "mapping.h"
#include "mm_model.h"
#include "mapping.h"
#include <boost/thread.hpp>

using namespace map;
class RTree;
class Route;
class ShapefileGraph;
class wxImagePanel : public wxPanel {
    DECLARE_EVENT_TABLE()
public:
    wxImagePanel (wxFrame* parent);
    ~wxImagePanel();
    
    // paint event
    void paintEvent (wxPaintEvent & evt);
    void paintNow();
    void OnSize (wxSizeEvent& event);
    void OnEraseBackground (wxEraseEvent& event);

    void render (wxDC& dc);
    bool InitCache();
    void DoCache (wxDC&  dc);
    void MuiltyThreadCache (wxDC& dc);
    
    // some useful events
    void mouseMoved (wxMouseEvent& event);
    void mouseDown (wxMouseEvent& event);
    void mouseWheelMoved (wxMouseEvent& event);
    void mouseReleased (wxMouseEvent& event);
    void rightClick (wxMouseEvent& event);
    void mouseLeftWindow (wxMouseEvent& event);
    void keyPressed (wxKeyEvent& event);
    void keyReleased (wxKeyEvent& event);
    void mouseLeftDClick (wxMouseEvent& event);
    void mouseRightDClick (wxMouseEvent& event);

    // Map Operations
    bool LoadMapDefine (wxString map_folder);
    void SetResolution (int level);
    void update_display_screen_center (int direction, double offset);
    void KillThreads();
    wxPoint CalculatePos(double x, double y);
    void GetTileRange();
    
    
    void Reset();
    
    // MM Operations
    bool RL();
    bool LoadRoute(std::string filename);
    bool LoadDensityRoute(std::string filename);
    
    bool BuildRTree(std::string filename);
    bool BuildGraph(std::string filename);
    bool ShortestPath(std::string filename);
    void SaveGraph(std::string filename);
    void LoadGraph(std::string filename);
    bool LocatePoints(int elements);
    void CalculateGroundTruth();
    
    // Access
    inline int GetResolution() { return current_lod_; }
    inline wxRect2DDouble& GetDisplayBoundary() { return display_boundary_; }
    inline void SetThreadNum (int n) { specified_thread_number_ = n; }
    inline void SetOutputDir (wxString dir) { out_dir_ = dir; }
    inline wxString GetOutputDir() { return out_dir_; }
    inline std::string GetMapDirectory() { return map_dir_; }
    inline std::string GetOutDirectory() { return out_dir_; }
    inline int GetTileNum() { return tile_number_; }
    inline int GetTotalTiles() { return tile_number_; }
    inline int GetResolutionLevel() { return current_lod_; }
    void GetTilePos(int index, int& row, int& col);
    inline boost::shared_ptr<Lod> GetLod() { return lod_; }
    inline boost::shared_ptr<MapStyle> GetMapStyle() { return map_style_; }
private:
    // MM variables
	boost::shared_ptr<RTree> tree_;
    // Sparse route
	boost::shared_ptr<Route> route_;
    // Density route
    boost::shared_ptr<Route> density_route_;
    std::vector<int> ground_truth_;
    
    boost::shared_ptr<ShapefileGraph> shapefile_graph_;
    boost::shared_ptr<MM> mm_;
    std::vector<State> parsing_result_;
	int neighbers_;
	std::vector<BoostLineString> nearest_lines_;
    std::vector<std::vector<wxPoint2DDouble> > candinate_points_;
    std::vector<std::vector<int> > candinate_points_colors_;
    std::vector<std::vector<int> > candinate_points_ids_;

    // Map Display
    wxPoint2DDouble display_map_center_;
    wxPoint display_screen_center_;
    wxPoint2DDouble display_half_size_;
    wxRect2DDouble display_boundary_, map_actual_boundary_;
    wxRect2DDouble map2screen_ratio_;

    // Mapping engine
	boost::shared_ptr<Mapping> engine_;
    boost::thread_group threads_;
    int thread_number_;
    int specified_thread_number_;
    std::string out_dir_, map_dir_;
	int maximal_threads_, real_threads_;
    boost::shared_ptr<Lod> lod_;
    boost::shared_ptr<MapStyle> map_style_;
    int min_index_x_, min_index_y_, max_index_x_, max_index_y_;
    int tile_number_;
    int w_, h_;
    wxPoint clicked_;
    wxCoord image_pos_x_, image_pos_y_;
    int current_lod_;
    // ready for cache flag
    bool ready_for_cache_;
    //int current_style_version_;
    bool apply_;
};
#endif // MAP_CLIENT_H_
