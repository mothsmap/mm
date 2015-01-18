#ifndef __map__job__hh__
#define __map__job__hh__

#include "wx/wx.h"
#include "panel.h"
#include "mapping.h"
#include "boost/filesystem.hpp"
#include "boost/thread.hpp"
#include "boost/lexical_cast.hpp"
#include "wx/dcbuffer.h"

using namespace boost::filesystem;
using namespace map;
boost::shared_mutex mutex;

#define ALLOW_CACHE 1

class GUIJob {
public:
    GUIJob (wxImagePanel* panel, int total_tasks, int task_id) {
        total_tasks_ = total_tasks;
        task_id_ = task_id;
        total_tiles_ = panel->GetTotalTiles();
        map_dir_ = panel->GetMapDirectory();
        out_dir_ = panel->GetOutDirectory();
        display_boundary_ = panel->GetDisplayBoundary();
        panel_ = panel;
    }
    
    ~GUIJob() {
    }
    
    
    void CacheFeature() {
        int circle = 0;
        int current_tile_index = circle * total_tasks_ + task_id_;
        
        while (current_tile_index < total_tiles_) {
            Mapping engine(map_dir_, out_dir_);
            engine.SetOuterPixels(32);
            
            // lod
            engine.LoadLod();
            engine.SetBoundary(display_boundary_.GetLeft(), display_boundary_.GetBottom(), display_boundary_.GetRight(), display_boundary_.GetTop());
            
            // tile position
            int row, col;
            panel_->GetTilePos(current_tile_index, row, col);
            int level = panel_->GetResolutionLevel();
            engine.SetTile(level, row, col);
            
            std::string cache_file = engine.GetTileName();

            bool exit = boost::filesystem::exists (cache_file);
            if (!exit) {
                bool status = engine.LoadStyleFromFile(map_dir_ + "/map.json");
                engine.SetupDataConnection();
                
                bool result = engine.CacheTile(false);
                //std::cout << "Tile " << row << ", " << col << ": " << result << std::endl;
            }
            
            ++circle;
            current_tile_index = circle * total_tasks_ + task_id_;
        }
    }
    
protected:
    wxImagePanel* panel_;
    std::string map_dir_, out_dir_;
    wxRect2DDouble display_boundary_;
    int          total_tasks_;
    int          task_id_;
    int			 total_tiles_;
};
#endif
