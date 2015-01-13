#ifndef ROUTE_H_
#define ROUTE_H_

#include <string>
#include <vector>
#include "wx/wx.h"

// GPS points
class Route {
public:
    Route(void);
     ~Route(void);
    
    // Load data from shapefile
    bool Load(std::string);
    void Resample(double res);
    
    inline bool isEmpty() { return route_.size() <= 0; }
    inline const std::vector<wxPoint2DDouble>& getRoute() { return route_; }
    void getRouteVertex(int index, double& x, double& y);
    
    void Reset();
    
private:
    // taxi route points
    std::vector<wxPoint2DDouble> route_;
};

#endif

