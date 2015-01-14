#include "frame.h"
#include "image_panel.h"
#include "wx/spinctrl.h"
#include "wx/numdlg.h"
#include "wx/busyinfo.h"
#include "boost/filesystem.hpp"

// 菜单项ID
enum {
    TEXT_Main = wxID_HIGHEST + 1, // declares an id which will be used to call our button
    MENU_New,
    MENU_Open,
	MENU_OPEN_ROUTE,
    MENU_OPEN_DENSITY_ROUTE,
    MENU_Quit,
    MENU_About,

    MENU_THREAD_NUMBER,
    MENU_OUTPUT_DIR,

	MENU_BUILD_RTREE,
	MENU_FIND_LOCATION,
    MENU_FIND_GROUND_TRUTH,
	MENU_FIND_ROUTE,
	MENU_FIND_OPTIMAL_ROUTE,
    MENU_BUILD_GRAPH,
    MENU_SAVE_GRAPH,
    MENU_LOAD_GRAPH,
    MENU_SHORTEST_PATH,
    MENU_RL,
    MENU_CLEAR,
    
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_APPLY,
    BUTTON_ZOOM_IN,
    BUTTON_ZOOM_OUT,
    LEVELS_SPIN,
    
    KEY_STROKE
};

// 地图移动的步长（像素）
#define MOVE_TOL 50

// 事件表，将菜单项和响应函数绑定
BEGIN_EVENT_TABLE (Frame, wxFrame)
    EVT_MENU (MENU_New, Frame::OnNewFile)
    EVT_MENU (MENU_Open, Frame::OnOpenFile)
    EVT_MENU (MENU_About, Frame::OnAbout)
    EVT_MENU (MENU_Quit, Frame::OnQuit)
	EVT_MENU (MENU_BUILD_RTREE, Frame::OnBuildRtree)
    EVT_MENU (MENU_BUILD_GRAPH, Frame::OnBuildGraph)
    EVT_MENU (MENU_SAVE_GRAPH, Frame::OnSaveGraph)
    EVT_MENU (MENU_LOAD_GRAPH, Frame::OnLoadGraph)
    EVT_MENU (MENU_SHORTEST_PATH, Frame::OnShortestPath)
    EVT_MENU (MENU_RL, Frame::OnRL)
    EVT_MENU (MENU_CLEAR, Frame::OnClear)

	EVT_MENU (MENU_FIND_LOCATION, Frame::OnFindLocation)
	EVT_MENU (MENU_OPEN_ROUTE, Frame::OnOpenRoute)
    EVT_MENU (MENU_OPEN_DENSITY_ROUTE, Frame::OnOpenDensityRoute)
    EVT_MENU (MENU_THREAD_NUMBER, Frame::OnThreadNum)
    EVT_MENU (MENU_OUTPUT_DIR, Frame::OnOutputDir)
    EVT_MENU (MENU_FIND_GROUND_TRUTH, Frame::OnFindGroudTruth)
    EVT_BUTTON (BUTTON_LEFT, Frame::OnLeft)
    EVT_BUTTON (BUTTON_RIGHT, Frame::OnRight)
    EVT_BUTTON (BUTTON_UP, Frame::OnUp)
    EVT_BUTTON (BUTTON_DOWN, Frame::OnDown)
    EVT_BUTTON (BUTTON_ZOOM_IN, Frame::OnZoomIn)
    EVT_BUTTON (BUTTON_ZOOM_OUT, Frame::OnZoomOut)
    EVT_BUTTON (MENU_Open, Frame::OnOpenFile)
    EVT_MOUSEWHEEL (wxImagePanel::mouseWheelMoved)

//	EVT_ERASE_BACKGROUND(Frame::OnEraseBackground)
END_EVENT_TABLE()

// 窗体类构造函数
Frame::Frame (const wxString& title) : wxFrame (NULL, wxID_ANY, title) {
    // 初始化地图panel
    drawPane_ = new wxImagePanel (this);

    Connect(KEY_STROKE, wxEVT_KEY_DOWN, wxKeyEventHandler(Frame::OnKeyStroke));
    
    // 导航按钮
    wxBoxSizer* navigator_sizer = new wxBoxSizer (wxHORIZONTAL);
    wxPanel* nivagitor_panel = new wxPanel (this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSUNKEN_BORDER | wxTAB_TRAVERSAL);
    nivagitor_panel->SetBackgroundColour (wxColour ("WHITE"));
    
    wxSize button_size (60, 30);

    left_button_ = new wxButton (nivagitor_panel, BUTTON_LEFT, wxT("Left"), wxDefaultPosition, button_size);
    left_button_->SetToolTip (wxT("Move left"));
    left_button_->SetBackgroundColour(*wxBLUE);
    
    right_button_ = new wxButton (nivagitor_panel, BUTTON_RIGHT, wxString ("Right"), wxDefaultPosition, button_size);
    right_button_->SetToolTip (wxString ("Move right"));
    right_button_->SetBackgroundColour(*wxGREEN);
    
    up_button_ = new wxButton (nivagitor_panel, BUTTON_UP, wxString ("Up"), wxDefaultPosition, button_size);
    up_button_->SetToolTip (wxString ("Move up"));
    up_button_->SetBackgroundColour(*wxYELLOW);
    
    down_button_ = new wxButton (nivagitor_panel, BUTTON_DOWN, wxString ("Down"), wxDefaultPosition, button_size);
    down_button_->SetToolTip (wxString ("Move down"));
    down_button_->SetBackgroundColour(*wxCYAN);
    
    zoom_in_button_ = new wxButton (nivagitor_panel, BUTTON_ZOOM_IN, wxString ("Zoom In"), wxDefaultPosition, button_size);
    zoom_in_button_->SetToolTip (wxString ("Zoom in"));
    zoom_in_button_->SetBackgroundColour(*wxRED);
    
    zoom_out_button_ = new wxButton (nivagitor_panel, BUTTON_ZOOM_OUT, wxString ("Zoom out"), wxDefaultPosition, button_size);
    zoom_out_button_->SetToolTip (wxString ("Zoom out"));
    zoom_out_button_->SetBackgroundColour(*wxGREEN);
    
    navigator_sizer->Add (left_button_);
    navigator_sizer->Add (right_button_);
    navigator_sizer->Add (up_button_);
    navigator_sizer->Add (down_button_);
    navigator_sizer->Add (zoom_in_button_);
    navigator_sizer->Add (zoom_out_button_);
    
    promote_text_ = new wxStaticText (nivagitor_panel, wxID_STATIC, wxT ("Use left buttons to navigate, double click left mouse(righ) to zoom in(out)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE | wxST_NO_AUTORESIZE);
    navigator_sizer->Add (promote_text_, 1, wxCENTER);
    nivagitor_panel->SetSizer (navigator_sizer);
    nivagitor_panel->Fit();

    wxBoxSizer* map_sizer = new wxBoxSizer (wxVERTICAL);
    map_sizer->Add (drawPane_, 10, wxEXPAND | wxALL, 3);
    map_sizer->Add (nivagitor_panel, 0, wxGROW | wxALL, 3);

    SetSizer (map_sizer);
    this->Maximize();

    // 菜单项
    wxMenu* fileMenu = new wxMenu;
    fileMenu->Append (MENU_Open, wxT ("Open Map ..."), wxT ("Open an existing road network"));
    fileMenu->Append (MENU_OPEN_ROUTE, wxT ("Open sparse route..."), wxT ("Open an existing gps route"));
    fileMenu->Append(MENU_OPEN_DENSITY_ROUTE, wxT("Open density route..."), wxT("Open density route"));
    fileMenu->Append (MENU_Quit, wxT ("Quite"), wxT ("Quit this program"));

	wxMenu* mmMenu = new wxMenu;
	mmMenu->Append(MENU_BUILD_RTREE, wxT ("Build R Treee"), wxT ("Build RTree for the road network"));
    mmMenu->Append(MENU_BUILD_GRAPH, wxT("Build Graph"), wxT("Build Graph for the road network"));
    mmMenu->Append(MENU_FIND_GROUND_TRUTH, wxT("Find Ground Truth"), wxT("Find Ground Truth"));
	mmMenu->Append(MENU_FIND_LOCATION, wxT ("Find Candidate points..."), wxT ("Find candidate points"));
    mmMenu->Append(MENU_SHORTEST_PATH, wxT("Find Nearest route"), wxT("Find Shortest Path between candidate points"));
    mmMenu->Append(MENU_RL, wxT("Reinforcement Learning"), wxT("Learning the global optimal route"));
    mmMenu->Append(MENU_CLEAR, wxT("Clear result"), wxT("Clear result"));
    
    wxMenu* settingMenu = new wxMenu;
	settingMenu->Append (MENU_THREAD_NUMBER, wxT ("#Threads..."), wxT ("Set threads number"));
    settingMenu->Append (MENU_OUTPUT_DIR, wxT ("Cache directory..."), wxT ("Set output directory"));

    wxMenu* helpMenu = new wxMenu;
    helpMenu->Append (MENU_About, wxT ("&About...\tF1"), wxT ("Show about dialog"));

    main_menu_ = new wxMenuBar();
    main_menu_->Append (fileMenu, wxT ("File"));
	main_menu_->Append(mmMenu, wxT("MM"));
    main_menu_->Append (settingMenu, wxT ("Setting"));
    main_menu_->Append (helpMenu, wxT ("Help"));

    SetMenuBar (main_menu_);

    CreateStatusBar (2);
    SetStatusText (wxT ("~~~Welcome~~~Please open a map"));
}

void Frame::UpdateStatusBarText(wxString& context) {
    SetStatusText (context);
}

void Frame::UpdateErrorText(const wxString& context) {
    promote_text_->SetLabel (context);
}

void Frame::mouseWheelMoved(wxMouseEvent& event) {}

void Frame::OnAbout(wxCommandEvent& event) {
    wxMessageBox (wxT (""), wxT ("~~~MM~~~"), wxOK | wxICON_INFORMATION, this);
}

void Frame::OnQuit(wxCommandEvent& event) {
    Close() ;
}

void Frame::OnNewFile(wxCommandEvent& WXUNUSED (event)) {}

// 打开地图
void Frame::OnOpenFile(wxCommandEvent& WXUNUSED (event)) {
    // Get the Map path
    wxString defaultPath = wxT ("/");
    wxDirDialog dialog (this, wxT ("Open Map directory"), defaultPath, wxDD_NEW_DIR_BUTTON);

    if (dialog .ShowModal() == wxID_OK) {
        map_path_ = dialog.GetPath();

        std::string outpath = map_path_.ToStdString() + "/cache";

        if (!boost::filesystem::exists (outpath)) {
            boost::filesystem::create_directories (outpath);
        }

        drawPane_->SetOutputDir (map_path_ + "/cache");

        if (!drawPane_->LoadMapDefine (map_path_))
            return;
        
        SetStatusText("Please Open GPS　route");
    }
}

void Frame::OnCloseFile(wxCommandEvent& WXUNUSED (event)) {
}

void Frame::OnSaveFile(wxCommandEvent& WXUNUSED (event)) {
}

void Frame::OnSaveFileAs(wxCommandEvent& WXUNUSED (event)) {
    wxFileDialog *SaveDialog = new wxFileDialog (
        this, _ ("Save File As _?"), wxEmptyString, wxEmptyString,
        _ ("Json files (*.json)|*.json"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT, wxDefaultPosition);

    // Creates a Save Dialog with 4 file types
    if (SaveDialog->ShowModal() == wxID_OK) { // If the user clicked "OK"
        wxString map_define_path = SaveDialog->GetPath();
    }

    // Clean up after ourselves
    SaveDialog->Destroy();
}


void Frame::OnUp(wxCommandEvent& WXUNUSED (event)) {
    drawPane_->update_display_screen_center (1, -MOVE_TOL);
    drawPane_->Refresh();
}

void Frame::OnDown(wxCommandEvent& WXUNUSED (event)) {
    drawPane_->update_display_screen_center (1, MOVE_TOL);
    drawPane_->Refresh();
}

void Frame::OnLeft(wxCommandEvent& WXUNUSED (event)) {
    drawPane_->update_display_screen_center (0, -MOVE_TOL);
    drawPane_->Refresh();
}

void Frame::OnRight(wxCommandEvent& WXUNUSED (event)) {
    drawPane_->update_display_screen_center (0, MOVE_TOL);
    drawPane_->Refresh();
}

void Frame::OnZoomIn(wxCommandEvent& event) {
 //   std::cout << "Zoom in!\n";
    std::cout << drawPane_->GetResolution() << std::endl;
    drawPane_->SetResolution (drawPane_->GetResolution() + 1);
    drawPane_->Refresh();
}

void Frame::OnZoomOut(wxCommandEvent& event) {
 //   std::cout << "Zoom out!\n";
    std::cout << drawPane_->GetResolution() << std::endl;
    drawPane_->SetResolution (drawPane_->GetResolution() - 1);
    drawPane_->Refresh();
}

void Frame::OnThreadNum (wxCommandEvent& event) {
	wxNumberEntryDialog dialog(this, wxT ("set thread number."), wxT ("#Threads:"), wxT ("Threads"), 8, 1, 16);

    if (dialog .ShowModal() == wxID_OK) {
        drawPane_->SetThreadNum (dialog.GetValue());
    }
}

// 打开稀疏点轨迹
void Frame::OnOpenRoute(wxCommandEvent& event) {
	wxString defaultPath = wxT ("/");
	wxString caption = wxT("Open GPS route");
	wxString wildcard = wxT("SHP files(*.shp)|*.shp");
	
	wxFileDialog dialog (this, caption, wxEmptyString, wxEmptyString, wildcard);

    if (dialog .ShowModal() == wxID_OK) {
		bool status = drawPane_->LoadRoute (dialog.GetPath().ToStdString());
		if (!status)
			wxMessageBox(wxT("Open fail!"), wxT("Error"));
    }

	drawPane_->Refresh();
    SetStatusText("Data completed! Pease build R Tree");
}

// 打开稀疏点
void Frame::OnOpenDensityRoute(wxCommandEvent& event) {
    wxString defaultPath = wxT ("/");
    wxString caption = wxT("Open GPS route");
    wxString wildcard = wxT("SHP files(*.shp)|*.shp");
    
    wxFileDialog dialog (this, caption, wxEmptyString, wxEmptyString, wildcard);
    
    if (dialog .ShowModal() == wxID_OK) {
        bool status = drawPane_->LoadDensityRoute(dialog.GetPath().ToStdString());
        if (!status)
            wxMessageBox(wxT("Open fail!"), wxT("error"));
    }
    
    drawPane_->Refresh();
    //  SetStatusText("数据准备完毕！请建立R树");
}

// 寻找真实匹配路径
void Frame::OnFindGroudTruth(wxCommandEvent& event) {
    drawPane_->CalculateGroundTruth();
    drawPane_->Refresh();
}

// 建立R树
void Frame::OnBuildRtree(wxCommandEvent& event) {
    wxWindowDisabler disableAll;
    wxBusyInfo info(wxT("Building R Tree, this may take several seconds..."), this);
    bool status = drawPane_->BuildRTree (map_path_.ToStdString());
    if (!status) {
        wxMessageBox(wxT("fail!"), wxT("error"));
        SetStatusText("Build R Tree fail!");
    } else {
        SetStatusText("Building R Tree success! Please build graph");
    }
}

// 建立图
void Frame::OnBuildGraph(wxCommandEvent& event) {
    bool status = drawPane_->BuildGraph (map_path_.ToStdString());
    if (!status) {
        wxMessageBox(wxT("fail!"), wxT("error"));
        SetStatusText("Build Graph fail!");
    } else {
        wxMessageBox (wxT ("Build Graph success"), wxT ("Info"), wxOK | wxICON_INFORMATION, this);
        SetStatusText("Build graph success! Please find candidate points");
    }
}

// 寻找候选点集
void Frame::OnFindLocation(wxCommandEvent& event) {
	//wxNumberEntryDialog dialog(this, wxT("输入GPS误差（单位为米）"), wxT("GPS误差: "),
	//	wxT("寻找候选点的阈值设定"), 250, 0, 500);

	//if (dialog.ShowModal() == wxID_OK) {
	//	long value = dialog.GetValue();
    int value = 5;
		if (!drawPane_->LocatePoints(value))
			wxMessageBox(wxT("No line near to a point!"), wxT("Warnning"));
        
        wxMessageBox (wxT ("Find candidate points success!"), wxT ("Info"), wxOK | wxICON_INFORMATION, this);
        
		drawPane_->Refresh();
	//}
    
    SetStatusText("Find candidate points success! Please find nearest routes");
}

void Frame::OnOutputDir (wxCommandEvent& event)
{
    // Get the Map path
    wxString defaultPath = wxT ("/");
    wxDirDialog dialog (this, wxT ("Output directory picker"), defaultPath, wxDD_NEW_DIR_BUTTON);

    if (dialog .ShowModal() == wxID_OK) {
        drawPane_->SetOutputDir (dialog.GetPath());
    }
}

// 寻找候选边集
void Frame::OnShortestPath(wxCommandEvent& event) {
    if (!drawPane_->ShortestPath(map_path_.ToStdString())) {
        wxMessageBox(wxT("fail!"), wxT("Warning"));
        SetStatusText("Calculate shortest path fail!");
    } else {
        wxMessageBox (wxT ("success!"), wxT ("Info"), wxOK | wxICON_INFORMATION, this);
        SetStatusText("find nearest routes success! Please start reinforcement learning");
    }
}
void Frame::OnSaveGraph(wxCommandEvent& event) {
    drawPane_->SaveGraph(map_path_.ToStdString());
}
void Frame::OnLoadGraph(wxCommandEvent& event) {
    drawPane_->LoadGraph(map_path_.ToStdString());
}

// 强化学习
void Frame::OnRL(wxCommandEvent& event) {
    wxWindowDisabler disableAll;
    wxBusyInfo info(wxT("Learning..."), this);
    
    bool result = drawPane_->RL();
    
    if (!result)
        wxMessageBox (wxT ("fail!"), wxT ("Info"), wxOK | wxICON_INFORMATION, this);
    
    drawPane_->Refresh();
    
    SetStatusText("Done");
}

void Frame::OnKeyStroke(wxKeyEvent& event) {
    if (event.GetKeyCode() == 37)
        drawPane_->update_display_screen_center(0, -MOVE_TOL);
    else if (event.GetKeyCode() == 39)
        drawPane_->update_display_screen_center(0, MOVE_TOL);
    else if (event.GetKeyCode() == 38)
        drawPane_->update_display_screen_center(1, -MOVE_TOL);
    else if (event.GetKeyCode() == 40)
        drawPane_->update_display_screen_center(1, MOVE_TOL);
    drawPane_->Refresh();
}

void Frame::OnClear(wxCommandEvent& event) {
    drawPane_->Reset();
    drawPane_->Refresh();
}


