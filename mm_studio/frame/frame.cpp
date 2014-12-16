#include "frame.h"
#include "image_panel.h"
#include "wx/spinctrl.h"
#include "wx/numdlg.h"
#include "wx/busyinfo.h"
#include "boost/filesystem.hpp"

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

#define MOVE_TOL 50

//IMPLEMENT_CLASS(Frame, wxFrame)

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

Frame::Frame (const wxString& title) : wxFrame (NULL, wxID_ANY, title) {
    drawPane_ = new wxImagePanel (this);

    Connect(KEY_STROKE, wxEVT_KEY_DOWN, wxKeyEventHandler(Frame::OnKeyStroke));
    
    wxBoxSizer* navigator_sizer = new wxBoxSizer (wxHORIZONTAL);
    wxPanel* nivagitor_panel = new wxPanel (this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSUNKEN_BORDER | wxTAB_TRAVERSAL);
    nivagitor_panel->SetBackgroundColour (wxColour ("WHITE"));
    
    wxSize button_size (60, 30);

    left_button_ = new wxButton (nivagitor_panel, BUTTON_LEFT, wxT("左移"), wxDefaultPosition, button_size);
    left_button_->SetToolTip (wxT("左移"));
    left_button_->SetBackgroundColour(*wxBLUE);
    
    right_button_ = new wxButton (nivagitor_panel, BUTTON_RIGHT, wxString ("右移"), wxDefaultPosition, button_size);
    right_button_->SetToolTip (wxString ("右移"));
    right_button_->SetBackgroundColour(*wxBLACK);
    
    up_button_ = new wxButton (nivagitor_panel, BUTTON_UP, wxString ("上移"), wxDefaultPosition, button_size);
    up_button_->SetToolTip (wxString ("上移"));
    up_button_->SetBackgroundColour(*wxYELLOW);
    
    down_button_ = new wxButton (nivagitor_panel, BUTTON_DOWN, wxString ("下移"), wxDefaultPosition, button_size);
    down_button_->SetToolTip (wxString ("下移"));
    down_button_->SetBackgroundColour(*wxCYAN);
    
    zoom_in_button_ = new wxButton (nivagitor_panel, BUTTON_ZOOM_IN, wxString ("放大"), wxDefaultPosition, button_size);
    zoom_in_button_->SetToolTip (wxString ("放大"));
    zoom_in_button_->SetBackgroundColour(*wxRED);
    
    zoom_out_button_ = new wxButton (nivagitor_panel, BUTTON_ZOOM_OUT, wxString ("缩小"), wxDefaultPosition, button_size);
    zoom_out_button_->SetToolTip (wxString ("缩小"));
    zoom_out_button_->SetBackgroundColour(*wxGREEN);
    
    navigator_sizer->Add (left_button_);
    navigator_sizer->Add (right_button_);
    navigator_sizer->Add (up_button_);
    navigator_sizer->Add (down_button_);
    navigator_sizer->Add (zoom_in_button_);
    navigator_sizer->Add (zoom_out_button_);
    
    promote_text_ = new wxStaticText (nivagitor_panel, wxID_STATIC, wxT ("使用左侧按钮导航地图，双击鼠标左键（右键）进行以鼠标位置为中心的放大（缩小）"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE | wxST_NO_AUTORESIZE);
    navigator_sizer->Add (promote_text_, 1, wxCENTER);
    nivagitor_panel->SetSizer (navigator_sizer);
    nivagitor_panel->Fit();

    wxBoxSizer* map_sizer = new wxBoxSizer (wxVERTICAL);
    map_sizer->Add (drawPane_, 10, wxEXPAND | wxALL, 3);
    map_sizer->Add (nivagitor_panel, 0, wxGROW | wxALL, 3);

    SetSizer (map_sizer);
    this->Maximize();

    wxMenu* fileMenu = new wxMenu;
    fileMenu->Append (MENU_Open, wxT ("打开路网 ..."), wxT ("Open an existing road network"));
    fileMenu->Append (MENU_OPEN_ROUTE, wxT ("打开稀疏点轨迹..."), wxT ("Open an existing gps route"));
    fileMenu->Append(MENU_OPEN_DENSITY_ROUTE, wxT("打开稠密点轨迹..."), wxT("打开稠密点轨迹"));
    fileMenu->Append (MENU_Quit, wxT ("退出"), wxT ("Quit this program"));

	wxMenu* mmMenu = new wxMenu;
	mmMenu->Append(MENU_BUILD_RTREE, wxT ("建立R树"), wxT ("Build RTree for the road network"));
    mmMenu->Append(MENU_BUILD_GRAPH, wxT("建立有向图"), wxT("Build Graph for the road network"));
    mmMenu->Append(MENU_FIND_GROUND_TRUTH, wxT("寻找真实路径"), wxT("从稠密点寻找真实路径"));
	mmMenu->Append(MENU_FIND_LOCATION, wxT ("寻找候选点..."), wxT ("Find candidate points"));
    mmMenu->Append(MENU_SHORTEST_PATH, wxT("寻找最短路径"), wxT("Find Shortest Path between candidate points"));
    mmMenu->Append(MENU_RL, wxT("增强学习"), wxT("Learning the global optimal route"));
    mmMenu->Append(MENU_CLEAR, wxT("清空结果"), wxT("清空当前计算结果"));
    
    wxMenu* settingMenu = new wxMenu;
    settingMenu->Append (MENU_THREAD_NUMBER, wxT ("线程数..."), wxT ("Set threads number"));
    settingMenu->Append (MENU_OUTPUT_DIR, wxT ("指定缓存目录..."), wxT ("Set output directory"));

    wxMenu* helpMenu = new wxMenu;
    helpMenu->Append (MENU_About, wxT ("&关于...\tF1"), wxT ("Show about dialog"));

    main_menu_ = new wxMenuBar();
    main_menu_->Append (fileMenu, wxT ("文件"));
	main_menu_->Append(mmMenu, wxT("地图匹配"));
    main_menu_->Append (settingMenu, wxT ("设置"));
    main_menu_->Append (helpMenu, wxT ("帮助"));

    SetMenuBar (main_menu_);

    CreateStatusBar (2);
    SetStatusText (wxT ("~~~欢迎~~~请从打开一个地图开始"));
}

void Frame::UpdateStatusBarText(wxString& context) {
    SetStatusText (context);
}

void Frame::UpdateErrorText(const wxString& context) {
    promote_text_->SetLabel (context);
}

void Frame::mouseWheelMoved(wxMouseEvent& event) {}

void Frame::OnAbout(wxCommandEvent& event) {
    wxMessageBox (wxT (""), wxT ("~~~地图匹配~~~"), wxOK | wxICON_INFORMATION, this);
}

void Frame::OnQuit(wxCommandEvent& event) {
    Close() ;
}

void Frame::OnNewFile(wxCommandEvent& WXUNUSED (event)) {}

void Frame::OnOpenFile(wxCommandEvent& WXUNUSED (event)) {
    // Get the Map path
    wxString defaultPath = wxT ("/");
    wxDirDialog dialog (this, wxT ("打开地图目录"), defaultPath, wxDD_NEW_DIR_BUTTON);

    if (dialog .ShowModal() == wxID_OK) {
        map_path_ = dialog.GetPath();

        std::string outpath = map_path_.ToStdString() + "/cache";

        if (!boost::filesystem::exists (outpath)) {
            boost::filesystem::create_directories (outpath);
        }

        drawPane_->SetOutputDir (map_path_ + "/cache");

        if (!drawPane_->LoadMapDefine (map_path_))
            return;
        
        SetStatusText("请打开GPS轨迹点");
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
    drawPane_->SetResolution (drawPane_->GetResolution() + 1);
    drawPane_->Refresh();
}

void Frame::OnZoomOut(wxCommandEvent& event) {
    drawPane_->SetResolution (drawPane_->GetResolution() - 1);
    drawPane_->Refresh();
}

void Frame::OnThreadNum (wxCommandEvent& event) {
    wxNumberEntryDialog dialog(this, wxT ("推荐的线程数为不超过20个."), wxT ("线程数:"), wxT ("设置线程数目"), 8, 1, 16);

    if (dialog .ShowModal() == wxID_OK) {
        drawPane_->SetThreadNum (dialog.GetValue());
    }
}

void Frame::OnOpenRoute(wxCommandEvent& event) {
	wxString defaultPath = wxT ("/");
	wxString caption = wxT("打开GPS轨迹的shapefile文件");
	wxString wildcard = wxT("SHP files(*.shp)|*.shp");
	
	wxFileDialog dialog (this, caption, wxEmptyString, wxEmptyString, wildcard);

    if (dialog .ShowModal() == wxID_OK) {
		bool status = drawPane_->LoadRoute (dialog.GetPath().ToStdString());
		if (!status)
			wxMessageBox(wxT("打开失败!"), wxT("错误"));
    }

	drawPane_->Refresh();
    SetStatusText("数据准备完毕！请建立R树");
}

void Frame::OnBuildRtree(wxCommandEvent& event) {
    wxWindowDisabler disableAll;
    wxBusyInfo info(wxT("建立R树中，请耐心等待..."), this);
    bool status = drawPane_->BuildRTree (map_path_.ToStdString());
    if (!status) {
        wxMessageBox(wxT("建立R树失败!"), wxT("错误"));
        SetStatusText("！建立R树失败！");
    } else {
        SetStatusText("建立R树成功！请建立有向图");
    }
}

void Frame::OnBuildGraph(wxCommandEvent& event) {
    bool status = drawPane_->BuildGraph (map_path_.ToStdString());
    if (!status) {
        wxMessageBox(wxT("建立有向图失败!"), wxT("错误"));
        SetStatusText("！建立有向图失败!");
    } else {
        wxMessageBox (wxT ("建立有向图成功！"), wxT ("提示"), wxOK | wxICON_INFORMATION, this);
        SetStatusText("建立有向图成功！请寻找GPS轨迹点的候选点");
    }
}

void Frame::OnFindLocation(wxCommandEvent& event) {
	//wxNumberEntryDialog dialog(this, wxT("输入GPS误差（单位为米）"), wxT("GPS误差: "),
	//	wxT("寻找候选点的阈值设定"), 250, 0, 500);

	//if (dialog.ShowModal() == wxID_OK) {
	//	long value = dialog.GetValue();
    int value = 5;
		if (!drawPane_->LocatePoints(value))
			wxMessageBox(wxT("No line near to a point!"), wxT("Warnning"));
        
        wxMessageBox (wxT ("寻找候选点成功！"), wxT ("提示"), wxOK | wxICON_INFORMATION, this);
        
		drawPane_->Refresh();
	//}
    
    SetStatusText("寻找候选点成功！请计算候选点之间的最短路径");
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

void Frame::OnShortestPath(wxCommandEvent& event) {
    if (!drawPane_->ShortestPath(map_path_.ToStdString())) {
        wxMessageBox(wxT("计算最短路径失败!"), wxT("警告"));
        SetStatusText("！计算最短路径失败！");
    } else {
        wxMessageBox (wxT ("计算最短路径成功！"), wxT ("提示"), wxOK | wxICON_INFORMATION, this);
        SetStatusText("计算最短路径成功！请开始增强学习得到匹配轨迹！");
    }
}
void Frame::OnSaveGraph(wxCommandEvent& event) {
    drawPane_->SaveGraph(map_path_.ToStdString());
}
void Frame::OnLoadGraph(wxCommandEvent& event) {
    drawPane_->LoadGraph(map_path_.ToStdString());
}

void Frame::OnRL(wxCommandEvent& event) {
    wxWindowDisabler disableAll;
    wxBusyInfo info(wxT("增强学习求解中，请耐心等待..."), this);
    
    bool result = drawPane_->RL();
    
    if (!result)
        wxMessageBox (wxT ("初始化增强学习模型失败！"), wxT ("提示"), wxOK | wxICON_INFORMATION, this);
    
    drawPane_->Refresh();
    
    SetStatusText("增强学习结束");
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

void Frame::OnOpenDensityRoute(wxCommandEvent& event) {
    wxString defaultPath = wxT ("/");
    wxString caption = wxT("打开GPS轨迹的shapefile文件");
    wxString wildcard = wxT("SHP files(*.shp)|*.shp");
    
    wxFileDialog dialog (this, caption, wxEmptyString, wxEmptyString, wildcard);
    
    if (dialog .ShowModal() == wxID_OK) {
        bool status = drawPane_->LoadDensityRoute(dialog.GetPath().ToStdString());
        if (!status)
            wxMessageBox(wxT("打开失败!"), wxT("错误"));
    }
    
    drawPane_->Refresh();
  //  SetStatusText("数据准备完毕！请建立R树");
}

void Frame::OnFindGroudTruth(wxCommandEvent& event) {
    drawPane_->CalculateGroundTruth();
    drawPane_->Refresh();
}

