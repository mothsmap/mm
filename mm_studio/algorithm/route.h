#ifndef __route__hh__
#define __route__hh__

#include <string>
#include <vector>
#include "wx/wx.h"

// GPS points
class Route {
public:
    Route(void);
     ~Route(void);
    
    // 从shapefile中加载
    bool Load(std::string);
    
    // 插入顶点
    void InsertNode(double x, double y);
    
    // 重采样
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

