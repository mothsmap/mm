#include "image_panel.h"
#include "frame.h"
#include "gui_job.h"

#include "map_style.h"
#include "boost/filesystem.hpp"
#include "boost/thread.hpp"

#include "wx/dcbuffer.h"

#include <fstream>
#include <algorithm>
#include "rtree.h"
#include "route.h"
#include "mm_model.h"
#include "shapefile_graph.h"
#include "mapping.h"

using namespace boost::filesystem;

BEGIN_EVENT_TABLE (wxImagePanel, wxPanel)
// some useful events
EVT_MOTION (wxImagePanel::mouseMoved)
EVT_LEFT_DOWN (wxImagePanel::mouseDown)
EVT_LEFT_UP (wxImagePanel::mouseReleased)
EVT_RIGHT_DOWN (wxImagePanel::rightClick)
EVT_LEAVE_WINDOW (wxImagePanel::mouseLeftWindow)
EVT_KEY_DOWN (wxImagePanel::keyPressed)
EVT_KEY_UP (wxImagePanel::keyReleased)
EVT_MOUSEWHEEL (wxImagePanel::mouseWheelMoved)
EVT_LEFT_DCLICK (wxImagePanel::mouseLeftDClick)
EVT_RIGHT_DCLICK (wxImagePanel::mouseRightDClick)
EVT_ERASE_BACKGROUND (wxImagePanel::OnEraseBackground)
// catch paint events
EVT_PAINT (wxImagePanel::paintEvent)
//Size event
EVT_SIZE (wxImagePanel::OnSize)
END_EVENT_TABLE()


// 加载地图
bool wxImagePanel::LoadMapDefine(wxString map_folder) {
    //SetBackgroundStyle(wxBG_STYLE_CUSTOM);
    wxColour color ("#CCE8CF");
    SetBackgroundColour (color);
    
    map_dir_ = map_folder;
    out_dir_ = map_folder + "/cache";
    
    lod_ =  boost::shared_ptr<Lod>(new Lod);
    bool status = lod_->LoadFromFile(map_dir_ + "/lod.json");
    
    map_style_ = boost::shared_ptr<MapStyle>(new MapStyle);
    status = map_style_->LoadFromFile(map_dir_ + "/map.json");
    
    // When initialize loading, show level 0
    if (map_style_->GetJumpRes() > 0) {
        SetResolution (lod_->GetNearestLevel (map_style_->GetJumpRes()));
        
        display_map_center_.m_x = map_style_->GetJumpX();
        display_map_center_.m_y = map_style_->GetJumpY();
    } else {
        SetResolution (0);
        
        // And the screen center is the map center
        
        double min_x, max_x, min_y, max_y;
        lod_->GetMapBoundary(min_x, min_y, max_x, max_y);
        display_map_center_.m_x = (min_x + max_x) / 2.0;
        display_map_center_.m_y = (min_y + max_y) / 2.0;
    }
    
    apply_ = false;
    
    ready_for_cache_ = true;
    
    this->Refresh();
    
    return true;
}

bool wxImagePanel::BuildRTree(std::string filename) {
	// build spatial tree
	bool status = tree_->Build(filename);
	return status;
}

bool wxImagePanel::BuildGraph(std::string filename) {

    const std::vector<wxPoint2DDouble>& points = route_->getRoute();
    
    gps_extent_minx_ = points[0].m_x;
    gps_extent_miny_ = points[0].m_y;
    gps_extent_maxx_ = points[0].m_x;
    gps_extent_maxy_ = points[0].m_y;;

    for (int i = 0; i < points.size(); ++i) {
            gps_extent_minx_ = gps_extent_minx_ < points[i].m_x ? gps_extent_minx_ : points[i].m_x;
            gps_extent_miny_ = gps_extent_miny_ < points[i].m_y ? gps_extent_miny_ : points[i].m_y;
            gps_extent_maxx_ = gps_extent_maxx_ > points[i].m_x ? gps_extent_maxx_ : points[i].m_x;
            gps_extent_maxy_ = gps_extent_maxy_ > points[i].m_y ? gps_extent_maxy_ : points[i].m_y;
    }

    
    const double thd = 500;
    bool status = shapefile_graph_->Build(tree_, gps_extent_minx_ - thd, gps_extent_miny_ - thd, gps_extent_maxx_ + thd, gps_extent_maxy_ + thd);
    
    return status;
}

void wxImagePanel::SetResolution (int level) {
    if (level < 0)
        level = 0;
    
    if (level >= lod_->GetResolutionLevel())
        level = lod_->GetResolutionLevel() - 1;
    
 //   std::cout << "current level: " << level << std::endl;
    current_lod_ = level;
}

void wxImagePanel::GetTileRange() {
    lod_->CalculateTileIndex (current_lod_, display_boundary_.GetLeft(), display_boundary_.GetTop(), min_index_x_, min_index_y_);
    lod_->CalculateTileIndex (current_lod_, display_boundary_.GetRight(), display_boundary_.GetBottom(), max_index_x_, max_index_y_);
    
    tile_number_ = (max_index_x_ - min_index_x_ + 1) * (max_index_y_ - min_index_y_ + 1);
}

// some useful events
void wxImagePanel::mouseMoved (wxMouseEvent& event) {
#if 0
	current_point = -1;
    
	const std::vector<wxPoint2DDouble>& points = route_->getRoute();
	for (int i = 0; i < points.size(); ++i) {
		wxPoint pos = CalculatePos(points[i].m_x, points[i].m_y);
		if (abs(pos.x - event.m_x) < 3 && abs(pos.y - event.m_y) < 3) {
			if (current_point == i)
				return;
            
			nearest_lines_.clear();
			current_point = i;
            
			std::vector<Value> result = tree_->Query(BoostPoint(points[i].m_x, points[i].m_y), neighbers_);
			for (int j = 0; j < result.size(); ++j) {
				int geometry_id = result[j].second;
				BoostLineString line = tree_->GetGeometry(geometry_id);
				nearest_lines_.push_back(line);
			}
			this->Refresh();
		}
	}
#endif
}

void wxImagePanel::mouseDown (wxMouseEvent& event) {}
void wxImagePanel::mouseWheelMoved (wxMouseEvent& event) {}
void wxImagePanel::mouseReleased (wxMouseEvent& event) {}
void wxImagePanel::rightClick (wxMouseEvent& event) {}
void wxImagePanel::mouseLeftWindow (wxMouseEvent& event) {}
void wxImagePanel::keyPressed (wxKeyEvent& event){}
void wxImagePanel::keyReleased (wxKeyEvent& event) {}

void wxImagePanel::mouseLeftDClick (wxMouseEvent& event) {
    if (!ready_for_cache_)
        return;
    
    wxPoint new_center = event.GetPosition();
    
    display_map_center_.m_x = display_boundary_.GetLeft() + (lod_->GetResolutionByLevel(current_lod_) * new_center.x);
    display_map_center_.m_y = display_boundary_.GetTop() - (lod_->GetResolutionByLevel(current_lod_) * new_center.y);
    
    SetResolution (++current_lod_);
    
    this->Refresh();
}

void wxImagePanel::mouseRightDClick (wxMouseEvent& event) {
    if (!ready_for_cache_)
        return;
    
    wxPoint new_center = event.GetPosition();
    
    display_map_center_.m_x = display_boundary_.GetLeft() + (lod_->GetResolutionByLevel(current_lod_) * new_center.x);
    display_map_center_.m_y = display_boundary_.GetTop() - (lod_->GetResolutionByLevel(current_lod_) * new_center.y);
    
    SetResolution (--current_lod_);
    
    this->Refresh();
}

wxImagePanel::wxImagePanel(wxFrame* parent) : wxPanel (parent) {
    w_ = -1;
    h_ = -1;
    image_pos_x_ = 0;
    image_pos_y_ = 0;
    
    specified_thread_number_ = 20;
    
#ifdef WIN32
    out_dir_ = "C:/Users/Default/AppData/Roaming";
#else
    out_dir_ = "/tmp";
#endif
    
    engine_ = NULL;
    
    ready_for_cache_ = false;

    candinate_points_.clear();
	tree_ = boost::shared_ptr<RTree>(new RTree);
	route_ = boost::shared_ptr<Route>(new Route);
    density_route_ = boost::shared_ptr<Route>(new Route);
    shapefile_graph_ = boost::shared_ptr<ShapefileGraph>(new ShapefileGraph);
    mm_ = boost::shared_ptr<MM>(new MM);
}

wxImagePanel::~wxImagePanel() {
}

/*
 * Called by the system of by wxWidgets when the panel needs
 * to be redrawn. You can also trigger this call by
 * calling Refresh()/Update().
 */
void wxImagePanel::paintEvent (wxPaintEvent & evt) {
    // depending on your system you may need to look at double-buffered dcs
    wxPaintDC dc (this);
    
    // background
    wxColour backgroundColour = GetBackgroundColour();
    
    if (!backgroundColour.Ok())
        backgroundColour =
        wxSystemSettings::GetColour (wxSYS_COLOUR_3DFACE);
    
    dc.SetBrush (wxBrush (backgroundColour));
    dc.SetPen (wxPen (backgroundColour, 1));
    wxRect windowRect (wxPoint (0, 0), GetClientSize());
    dc.DrawRectangle (windowRect);
    
    // draw foreground
    render(dc);
}

/*
 * Alternatively, you can use a clientDC to paint on the panel
 * at any time. Using this generally does not free you from
 * catching paint events, since it is possible that e.g. the window
 * manager throws away your drawing when the window comes to the
 * background, and expects you will redraw it when the window comes
 * back (by sending a paint event).
 */
void wxImagePanel::paintNow() {
    // depending on your system you may need to look at double-buffered dcs
    wxClientDC dc (this);
    
    wxColour backgroundColour = GetBackgroundColour();
    
    if (!backgroundColour.Ok())
        backgroundColour =
        wxSystemSettings::GetColour (wxSYS_COLOUR_3DFACE);
    
    dc.SetBrush (wxBrush (backgroundColour));
    dc.SetPen (wxPen (backgroundColour, 1));
    wxRect windowRect (wxPoint (0, 0), GetClientSize());
    dc.DrawRectangle (windowRect);
    
    //wxBufferedDC dc(&client_dc);
    render (dc);
}

wxPoint wxImagePanel::CalculatePos(double x, double y) {
	double offset_x = x- display_boundary_.GetLeft();
	double offset_y = display_boundary_.GetTop() - y;
    
    int pos_x = offset_x / lod_->GetResolutionByLevel(current_lod_);
    int pos_y = offset_y / lod_->GetResolutionByLevel(current_lod_);
    
    return wxPoint(pos_x, pos_y);
}

/*
 * Here we do the actual rendering. I put it in a separate
 * method so that it can work no matter what type of DC
 * (e.g. wxPaintDC or wxClientDC) is used.
 */
void wxImagePanel::render (wxDC&  dc) {
    dc.GetSize (&w_, &h_);
#if 1
    // base map tile
    if (ready_for_cache_) {
        MuiltyThreadCache (dc);
    }
#endif
    // taxi route
    if (!density_route_->isEmpty()) {
        const std::vector<wxPoint2DDouble>& points = density_route_->getRoute();
        for (int i = 0; i < points.size(); ++i) {
            // calculate pos
            wxPoint pos = CalculatePos(points[i].m_x, points[i].m_y);
            
            dc.SetPen(*wxRED_PEN);
            dc.SetBrush(*wxYELLOW);
            dc.DrawCircle(pos, 1);
        }
    }
    
    if (!route_->isEmpty()) {
        const std::vector<wxPoint2DDouble>& points = route_->getRoute();
        for (int i = 0; i < points.size(); ++i) {
            // calculate pos
            wxPoint pos = CalculatePos(points[i].m_x, points[i].m_y);
            
            dc.SetPen(*wxGREEN_PEN);
            dc.SetBrush(*wxRED_BRUSH);
            dc.DrawCircle(pos, 5);
        }
    }

    if (ground_truth_.size() > 0 && !mm_->parsing_result_valid()) {
        wxPen pen(*wxBLUE_PEN);
        pen.SetWidth(3);
        
        for (int i = 0; i < ground_truth_.size(); ++i) {
            int geometry_id = ground_truth_[i];
            BoostLineString road = tree_->GetEdge(geometry_id);
            
            double x1 = road.at(0).get<0>();
            double y1 = road.at(0).get<1>();
            double x2 = road.at(1).get<0>();
            double y2 = road.at(1).get<1>();
            
            wxPoint pos1 = CalculatePos(x1, y1);
            wxPoint pos2 = CalculatePos(x2, y2);
            
            dc.SetPen(pen);
            dc.DrawLine(pos1, pos2);
        }
    }

    
    // parsing result
    if (mm_->parsing_result_valid()) {
        wxPen pen(*wxCYAN);
        pen.SetWidth(3);
        dc.SetPen(pen);
        
#if PRINT_INFO
        std::ofstream ofs(map_dir_ + "/result.txt");
#endif
        std::vector<Action>& parsing_actions = mm_->get_parsing_action();
        
#if PRINT_INFO
        std::cout << "Actions: ";
#endif
        for (int i = 1; i < parsing_actions.size(); ++i) {
#if PRINT_INFO
            std::cout << "from gps id: " << parsing_actions[i].from_vertex_gps_id_ << " to gps id: " << parsing_actions[i].to_vertex_gps_id_ << std::endl;
            
            ofs << "(" << parsing_actions[i].from_vertex_gps_id_ << ", " << parsing_actions[i].from_vertex_candidate_id_ << ") -> (" << parsing_actions[i].to_vertex_gps_id_ << ", " << parsing_actions[i].to_vertex_candidate_id_ << ")" << std::endl;
#endif
            std::vector<BoostPoint>& nodes = parsing_actions[i].nodes_;
            std::vector<wxPoint> points_in_path_of_screen;
            
            if (nodes.size() == 0) {
                std::cout << "Not good nodes!\n";
                continue;
            }
            
            ofs << "road id";
            for (int j = 0; j < nodes.size(); ++j) {
                ofs << parsing_actions[i].path_[j] << "\t";
                
                wxPoint pos = CalculatePos(nodes[j].get<0>(), nodes[j].get<1>());
                points_in_path_of_screen.push_back(pos);
            }
            
            
            for (int j = 0; j < points_in_path_of_screen.size() - 1; j += 2) {
                dc.DrawLine(points_in_path_of_screen[j], points_in_path_of_screen[j + 1]);
            }
        }
        
#if PRINT_INFO
        ofs.close();
#endif
    } else {
        // candinate points
        for (int i = 0; i < candinate_points_.size(); ++i) {
            std::vector<wxPoint2DDouble> points = candinate_points_[i];
            
            wxColour color(candinate_points_colors_[i][0], candinate_points_colors_[i][1],
                           candinate_points_colors_[i][2]);
            wxPen pen(color);
            
            wxBrush brush(color);
            dc.SetBrush(brush);
            
            for (int j = 0; j < points.size(); ++j){
                wxPoint pos = CalculatePos(points[j].m_x, points[j].m_y);
                
                dc.SetPen(*wxBLUE_PEN);
                dc.DrawCircle(pos, 3);
            }
        }
    }
}

void wxImagePanel::KillThreads() {
    threads_.interrupt_all();
    threads_.join_all();
}

void wxImagePanel::MuiltyThreadCache (wxDC& dc) {
    display_half_size_.m_x = lod_->GetResolutionByLevel(current_lod_) * (w_ / 2);
    display_half_size_.m_y = lod_->GetResolutionByLevel(current_lod_) * (h_ / 2);
    
    display_boundary_.SetLeft (display_map_center_.m_x - display_half_size_.m_x);
    display_boundary_.SetRight (display_map_center_.m_x + display_half_size_.m_x);
    display_boundary_.SetTop (display_map_center_.m_y + display_half_size_.m_y);
    display_boundary_.SetBottom (display_map_center_.m_y - display_half_size_.m_y);
    
    // set cache boundary
    //	engine_->SetBoundary(display_boundary_.GetLeft(), display_boundary_.GetBottom(), display_boundary_.GetRight(), display_boundary_.GetTop());
    
    // Get all the tiles needed to cache
    this->GetTileRange();
    
    maximal_threads_ = specified_thread_number_;
    
    int tile_number = this->GetTileNum();
    
    real_threads_ = tile_number > maximal_threads_ ? maximal_threads_ : tile_number;
    
    if (threads_.size() > 0) {
        KillThreads();
    }
    
    for (int i = 0; i < real_threads_; ++i) {
        boost::shared_ptr<GUIJob> job = boost::shared_ptr<GUIJob> (new GUIJob (this, real_threads_, i));
        boost::thread* thread = new boost::thread (boost::bind (&GUIJob::CacheFeature, job));
        threads_.add_thread(thread);
    }
    
    threads_.join_all();
    
    for (int i = 0; i < tile_number; ++i) {
        int row, col;
        double xmin, ymin, xmax, ymax;
        
        this->GetTilePos(i, row, col);
        double original_x, original_y, extent;
        lod_->CalculateTileExtent(current_lod_, row, col, original_x, original_y, extent);
        xmin = original_x;
        ymin = original_y - extent;
        xmax = original_x + extent;
        ymax = original_y;
        
        
        wxPoint2DDouble tile_offset;
        wxRect2DDouble& display_boundary = this->GetDisplayBoundary();
        
        tile_offset.m_x = xmin- display_boundary.GetLeft();
        tile_offset.m_y = display_boundary.GetTop() - ymax;
        
        int image_pos_x = tile_offset.m_x / lod_->GetResolutionByLevel(current_lod_);
        int image_pos_y = tile_offset.m_y / lod_->GetResolutionByLevel(current_lod_);
        
 //       std::cout << "image x: " << image_pos_x << ", image y: " << image_pos_y << std::endl;
        std::string tile_path = out_dir_ + "/" + boost::lexical_cast<std::string>(current_lod_)
        + "_" + boost::lexical_cast<std::string>(row) + "_" + boost::lexical_cast<std::string>(col) + ".png";
 //       std::cout << "tile path: " << tile_path << std::endl;
        bool file_exits = boost::filesystem::exists (tile_path);
        if (file_exits) {
            wxImage tile_png;
            tile_png.LoadFile (tile_path, wxBITMAP_TYPE_PNG);
            dc.DrawBitmap (tile_png, image_pos_x, image_pos_y, true);
        }
    }
    
    apply_ = false;
}

/*
 * Here we call refresh to tell the panel to draw itself again.
 * So when the user resizes the image panel the image should be resized too.
 */
void wxImagePanel::OnSize (wxSizeEvent& event) {
    Refresh();
    //skip the event.
    event.Skip();
}

void wxImagePanel::update_display_screen_center (int direction, double offset) {
    if (direction == 0) {
        display_map_center_.m_x += offset * lod_->GetResolutionByLevel(current_lod_);
    } else if (direction == 1) {
        display_map_center_.m_y -= offset * lod_->GetResolutionByLevel(current_lod_);
    }
}

bool wxImagePanel::LoadRoute(std::string filename) {
    route_->Reset();
    if (!route_->Load(filename)) {
        return false;
    }
    
    return true;
}

bool wxImagePanel::LoadDensityRoute(std::string filename) {
    density_route_->Reset();
    if (!density_route_->Load(filename)) {
        return false;
    }
    
    // resample
    density_route_->Resample(15);
    return true;
}

void wxImagePanel::GetProjectPoint(double x1, double y1, double x2, double y2, double x, double y, double& xx, double& yy) {
    // 两个端点在x-y方向的最大最小值
    double xmin = x1 < x2 ? x1 : x2;
    double xmax = x1 > x2 ? x1 : x2;
    double ymin = y1 < y2 ? y1 : y2;
    double ymax = y1 > y2 ? y1 : y2;
    
    // x最小值对应的y
    double xmin2y = x1 < x2 ? y1 : y2;
    // x最大值对应的y
    double xmax2y = x1 > x2 ? y1 : y2;
    
    // 阈值
    const double error_value = 0.0000001;
    
    // 直线参数 y = k * x + b
    double k, b;
    
    // 如果线段平行于X轴
    if (abs(y2 - y1) < error_value) {
        if (x <= xmin)
            xx = xmin;
        else if (x >= xmax)
            xx = xmax;
        else
            xx = x;
        yy = y1;
        
    } else if (abs(x2 - x1) < error_value) {
        // 如果线段平行于y轴
            if (y <= ymin)
                yy = ymin;
            else if (y >= ymax)
                yy = ymax;
            else
                yy = y;
            
            xx = x1;
        } else {
            // 如果线段不平行于x、y轴
            k = (y2 - y1) / (x2 - x1);
            b = y2 - k * x2;
            
            xx = (k * y + x - k * b) / (k * k + 1);
            yy = k * xx + b;
            
            if (xx <= xmin) {
                xx = xmin;
                yy = xmin2y;
            } else if (xx >= xmax) {
                xx = xmax;
                yy = xmax2y;
            } else {
                
            }
        }
}

bool wxImagePanel::LocatePoints(int elements) {
	neighbers_ = elements;
	nearest_lines_.clear();
    candinate_points_.clear();

	const std::vector<wxPoint2DDouble>& points = route_->getRoute();
	for (int i = 0; i < points.size(); ++i) {
        std::vector<Value> result = tree_->Query(EDGE, BoostPoint(points[i].m_x, points[i].m_y), elements);
        // 上海数据的误差
        //double radius = 250;
        //std::vector<Value> result = tree_->Query(EDGE, points[i].m_x - elements, points[i].m_y - elements, points[i].m_x + elements, points[i].m_y + elements);
		if (result.size() == 0)
			return false;
        
        // 点i的候选点
		std::vector<wxPoint2DDouble> candinate_points;
        std::vector<int> colors;
        std::vector<int> candinate_points_ids;
		for (int j = 0; j < result.size(); ++j) {
			int geometry_id = result[j].second;
			BoostLineString line = tree_->GetEdge(geometry_id);
            RoadInfo info = tree_->GetRoadInfo(geometry_id);
            
            // 线段两个端点
            double x1 = line.at(0).get<0>();
            double y1 = line.at(0).get<1>();
            double x2 = line.at(1).get<0>();
            double y2 = line.at(1).get<1>();
            double xx, yy;
            GetProjectPoint(x1, y1, x2, y2, points[i].m_x, points[i].m_y, xx, yy);
            
            candinate_points.push_back(wxPoint2DDouble(xx, yy));
            
            colors.push_back(rand() % 255);
            colors.push_back(rand() % 255);
            colors.push_back(rand() % 255);
            candinate_points_ids.push_back(geometry_id);
            
            shapefile_graph_->AddCandidatePoint(x1, y1, x2, y2, xx, yy, i, j, info.oneway_, geometry_id);
		}
        
        candinate_points_.push_back(candinate_points);
        candinate_points_colors_.push_back(colors);
        candinate_points_ids_.push_back(candinate_points_ids);
	}
    
	return true;
}

void wxImagePanel::CalculateGroundTruth() {
    // score function
    // s = sd + sa
    // sd(pi, ci) = ud - a * d(pi, ci)^nd
    // sa(pi, ci) = ua * cos(ai,j)^na
    double ud = 10;
    double a = 0.17;
    double nd = 1.4;
    
    double ua = 10;
    double na = 4;
    
    ground_truth_.clear();
    const std::vector<wxPoint2DDouble>& points = density_route_->getRoute();
    
    for (int i = 0; i < points.size(); ++i) {
        // Find nearest road to this point
        //std::vector<Value> result = tree_->Query(EDGE, BoostPoint(points[i].m_x, points[i].m_y), 5);
        int step_size = 50;
        std::vector<Value> result = tree_->Query(EDGE, points[i].m_x - step_size, points[i].m_y - step_size, points[i].m_x + step_size, points[i].m_y + step_size);
        while (result.size() <= 0) {
            step_size += 5;
            result = tree_->Query(EDGE, points[i].m_x - step_size, points[i].m_y - step_size, points[i].m_x + step_size, points[i].m_y + step_size);
        }
        
        int best_geometry = result[0].second;
        double best_score = 0;
        
        for (int j = 0; j < result.size(); ++j) {
            int geometry_id = result[j].second;
            BoostLineString line = tree_->GetEdge(geometry_id);
            
            double x1 = line.at(0).get<0>();
            double y1 = line.at(0).get<1>();
            double x2 = line.at(1).get<0>();
            double y2 = line.at(1).get<1>();
            double xx, yy;
            GetProjectPoint(x1, y1, x2, y2, points[i].m_x, points[i].m_y, xx, yy);
            
            double sd = 0;
            double dist = sqrt((points[i].m_x - xx) * (points[i].m_x - xx) + (points[i].m_y - yy) * (points[i].m_y - yy));
            sd = ud - a * pow(dist, nd);
            
            double sa = 0;
            if (i != 0) {
                double u = x2 - x1;
                double v = y2 - y1;
                double m = points[i].m_x - points[i - 1].m_x;
                double n = points[i].m_y - points[i - 1].m_y;
                double cos_theta = (u * m + v * n) / (sqrt(u * u + v * v) * sqrt(m * m + n * n));
                if (cos_theta < 0)
                    cos_theta = -cos_theta;
                
                sa = ua * pow(cos_theta, na);
            }
    
            double s = sd + sa;
            if (best_score < s) {
                best_score = s;
                best_geometry = geometry_id;
            }
        }
        
        if (ground_truth_.size() == 0) {
            ground_truth_.push_back(best_geometry);
        } else {
            if (best_geometry != ground_truth_[ground_truth_.size() - 1])
                ground_truth_.push_back(best_geometry);
        }
    }
    
    // 更新历史经验库
    for (int i = 0; i < ground_truth_.size(); ++i) {
        tree_->TravelEdge(ground_truth_[i]);
    }
}

bool wxImagePanel::ShortestPath(std::string filename) {
    bool result = shapefile_graph_->ComputeShortestPath(filename);
    
    if (!result) {
        return false;
    } else {
       // shapefile_graph_->NormalizeShortestPathLengths();
        return true;
    }
}

void wxImagePanel::SaveGraph(std::string filename) {
    shapefile_graph_->Save(filename);
}

void wxImagePanel::LoadGraph(std::string filename) {
    shapefile_graph_->Load(filename);
}

void wxImagePanel::OnEraseBackground (wxEraseEvent& event) {}

bool wxImagePanel::RL() {
    if (!mm_->InitModel(tree_, route_, candinate_points_, shapefile_graph_->GetShortestPaths()))
        return false;
    
    double score = 0;
    score = mm_->RunParsingAlgorithm(500000, 100000, 0.1);
//    int count = 0;
//    while (!mm_->parsing_result_valid() && count < 10) {
//        score = mm_->RunParsingAlgorithm(30000, 10000, 0.1);
//        ++count;
 //   }
    
    ((Frame*)this->GetParent())->UpdateErrorText("强化学习得分：" + boost::lexical_cast<std::string>(score));
    
   // return count < 10;
    return true;
}

void wxImagePanel::Reset() {
    route_->Reset();
    shapefile_graph_->Reset();
    mm_->Clear();
    parsing_result_.clear();
    nearest_lines_.clear();
    candinate_points_.clear();
    candinate_points_colors_.clear();
    candinate_points_ids_.clear();

}

void wxImagePanel::GetTilePos(int index, int& row, int& col) {
    int length = max_index_y_ - min_index_y_ + 1;
    row = min_index_x_ + index / length;
    col = min_index_y_ + index % length;
}
