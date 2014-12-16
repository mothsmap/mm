#include "route.h"
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

void Route::getRouteVertex(int index, double& x, double& y) {
    x = route_[index].m_x;
    y = route_[index].m_y;
}
