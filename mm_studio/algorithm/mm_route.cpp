#include "mm_route.h"
#include "ogrsf_frmts.h"

Route::Route(void) {
}

Route::~Route(void) {
}

void Route::Reset() {
    route_.clear();
}

bool Route::Load(std::string filename) {
    route_.clear();
    
    OGRRegisterAll();
    OGRDataSource* data_source = OGRSFDriverRegistrar::Open(filename.c_str(), FALSE);
    if (data_source == NULL) {
        return false;
    }
    
    OGRLayer* layer = data_source->GetLayer(0);
    OGRFeature* feature = layer->GetNextFeature();
    while (feature != NULL) {
        // feature geometry
        OGRGeometry *poGeometry;
        poGeometry = feature->GetGeometryRef();
        OGRPoint *poPoint = (OGRPoint*) poGeometry;
        
        route_.push_back(wxPoint2DDouble(poPoint->getX(), poPoint->getY()));
        
        OGRFeature::DestroyFeature (feature);
        feature = layer->GetNextFeature();
    }
    
    return true;
}

void Route::InsertNode(double x, double y) {
    route_.push_back(wxPoint2DDouble(x, y));
}

void Route::Resample(double res) {
    std::vector<wxPoint2DDouble> resampled_route_;
    resampled_route_.push_back(route_[0]);
    wxPoint2DDouble pt_pre = route_[0];
    
    std::cout << "before resample: " << route_.size() << std::endl;
    
    for (int i = 1; i < route_.size(); ++i) {
        wxPoint2DDouble pt = route_[i];
        double distance = sqrt((pt.m_x - pt_pre.m_x) * (pt.m_x - pt_pre.m_x) + (pt.m_y - pt_pre.m_y) * (pt.m_y - pt_pre.m_y));
        if (distance > res) {
            resampled_route_.push_back(pt);
            pt_pre = pt;
        }
    }
    
    // 更新route
    route_.clear();
    for (int i = 0; i < resampled_route_.size(); ++i) {
        route_.push_back(resampled_route_[i]);
    }
    
    std::cout << "after resample: " << route_.size() << std::endl;
}

void Route::getRouteVertex(int index, double& x, double& y) {
    x = route_[index].m_x;
    y = route_[index].m_y;
}