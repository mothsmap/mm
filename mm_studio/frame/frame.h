#ifndef __frame__hh__
#define __frame__hh__

#include "wx/wx.h"

class wxImagePanel;
class wxStaticText;
class wxSpinCtrl;
class Frame: public wxFrame
{
    DECLARE_EVENT_TABLE()
public:
    Frame (const wxString& title);

    void OnQuit (wxCommandEvent& event);
    void OnAbout (wxCommandEvent& event);
    //void OnEraseBackground(wxEraseEvent& event);

    void mouseWheelMoved (wxMouseEvent& event);

    void OnNewFile (wxCommandEvent& event);
    void OnOpenFile (wxCommandEvent& event);
	void OnOpenRoute(wxCommandEvent& event);
    void OnOpenDensityRoute(wxCommandEvent& event);
    void OnFindGroudTruth(wxCommandEvent& event);
	void OnFindLocation(wxCommandEvent& event);
	void OnBuildRtree(wxCommandEvent& event);
    
    void OnShortestPath(wxCommandEvent& event);
    void OnSaveGraph(wxCommandEvent& event);
    void OnLoadGraph(wxCommandEvent& event);
    void OnBuildGraph(wxCommandEvent& event);
    void OnRL(wxCommandEvent& event);
    void OnClear(wxCommandEvent& event);
    void OnSaveFile (wxCommandEvent& event);
    void OnSaveFileAs (wxCommandEvent& event);
    void OnCloseFile (wxCommandEvent& event);
    void OnThreadNum (wxCommandEvent& event);
    void OnOutputDir (wxCommandEvent& event);

    void OnKeyStroke(wxKeyEvent& event);
    void OnLeft (wxCommandEvent& event);
    void OnRight (wxCommandEvent& event);
    void OnUp (wxCommandEvent& event);
    void OnDown (wxCommandEvent& event);
    void OnZoomIn (wxCommandEvent& event);
    void OnZoomOut (wxCommandEvent& event);
    void UpdateStatusBarText (wxString& context);
    void UpdateErrorText (const wxString& context);

    void SetSpinValue (int value);
    void ChangeButtonStatus (bool disable);

private:
    // controls
    wxImagePanel* drawPane_;
    wxMenuBar *main_menu_;
    wxMenuItem* open_map_menu_, *open_gps_points_menu_, *new_map_menu_;
    wxStaticText* error_text_, *promote_text_;
    wxButton* left_button_, *right_button_, *up_button_, *down_button_, *zoom_in_button_, *zoom_out_button_;
    
    // Map configure
    wxString map_path_;
    wxRect2DDouble map_boundary_;
    wxVector<double> resolutions_;
};

#endif // _MYFRAME_H_

