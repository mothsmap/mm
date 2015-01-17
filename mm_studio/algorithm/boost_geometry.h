#ifndef USE_BOOST_FILES_H
#define USE_BOOST_FILES_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/multi/geometries/multi_geometries.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/variant.hpp>

#include <vector>
#include <map>
#include <string>
#include <iostream>

#define PRINT_INFO 1
#define VERY_SMALL_NUMBER 0.001

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 2, bg::cs::cartesian> BoostPoint;
typedef bg::model::segment<BoostPoint> BoostSegment;
typedef bg::model::box<BoostPoint> BoostBox;
typedef bg::model::polygon<BoostPoint, false, false> BoostPolygon; // ccw, open polygon
typedef bg::model::ring<BoostPoint, false, false> BoostRing; // ccw, open ring
typedef bg::model::linestring<BoostPoint> BoostLineString;;

typedef boost::variant<
BoostPoint,
BoostLineString,
BoostPolygon
> Boost_Geometry;

// typedef std::map<int, BoostLineString> GeometryMap;
typedef std::pair<BoostBox, int> Value;
typedef bgi::rtree<Value, bgi::rstar<16>> Boost_RTree;

struct envelope_visitor : public boost::static_visitor<BoostBox> {
    BoostBox operator()(BoostPoint const& g) const { return bg::return_envelope<BoostBox>(g); }
    BoostBox operator()(BoostLineString const& g) const { return bg::return_envelope<BoostBox>(g); }
    BoostBox operator()(BoostPolygon const& g) const { return bg::return_envelope<BoostBox>(g); }
};

class GeometryUtility {
public:
    static double Distance(double x1, double y1, double x2, double y2) {
        double dist_x = x1 - x2;
        double dist_y = y1 - y2;
        
        double dist = dist_x * dist_x + dist_y * dist_y;
        
        return std::sqrt(dist);
    }
    
    static bool PointEqual(double x1, double y1, double x2, double y2) {
        return (fabs(x1 - x2) <= VERY_SMALL_NUMBER && fabs(y1 - y2) <= VERY_SMALL_NUMBER);
    }
    
    static bool LineEqual(BoostLineString& line1, BoostLineString& line2) {
        double x1 = line1.at(0).get<0>();
        double y1 = line1.at(0).get<1>();
        double x2 = line1.at(1).get<0>();
        double y2 = line1.at(1).get<1>();
        
        double m1 = line2.at(0).get<0>();
        double n1 = line2.at(0).get<1>();
        double m2 = line2.at(1).get<0>();
        double n2 = line2.at(1).get<1>();
        
        if (PointEqual(x1, y1, m1, n1) && PointEqual(x2, y2, m2, n2)) {
            return true;
        }
        
        if (PointEqual(x1, y1, m2, n2) && PointEqual(x2, y2, m1, n1)) {
            return true;
        }
        
        return false;
    }
    
    static bool GetProjectPoint(double x1, double y1, double x2, double y2, double x, double y, double& xx, double& yy) {
        // 两个端点在x-y方向的最大最小值
        double xmin = x1 < x2 ? x1 : x2;
        double xmax = x1 > x2 ? x1 : x2;
        double ymin = y1 < y2 ? y1 : y2;
        double ymax = y1 > y2 ? y1 : y2;
        
        // x最小值对应的y
        double xmin2y = x1 < x2 ? y1 : y2;
        // x最大值对应的y
        double xmax2y = x1 > x2 ? y1 : y2;
        
        // 直线参数 y = k * x + b
        double k, b;
        
        bool inside = true;
        // 如果线段平行于X轴
        if (abs(y2 - y1) < VERY_SMALL_NUMBER) {
            if (x <= xmin) {
                xx = xmin;
                inside = false;
            } else if (x >= xmax) {
                inside = false;
                xx = xmax;
            } else {
                inside = true;
                xx = x;
            }
            yy = y1;
            
        } else if (abs(x2 - x1) < VERY_SMALL_NUMBER) {
            // 如果线段平行于y轴
            if (y <= ymin) {
                inside = false;
                yy = ymin;
            } else if (y >= ymax) {
                inside = false;
                yy = ymax;
            } else {
                inside = true;
                yy = y;
            }
            xx = x1;
        } else {
            // 如果线段不平行于x、y轴
            k = (y2 - y1) / (x2 - x1);
            b = y2 - k * x2;
            
            xx = (k * y + x - k * b) / (k * k + 1);
            yy = k * xx + b;
            
            if (xx <= xmin) {
                inside = false;
                xx = xmin;
                yy = xmin2y;
            } else if (xx >= xmax) {
                inside = false;
                xx = xmax;
                yy = xmax2y;
            } else {
                inside = true;
            }
        }
        
        return inside;
    }
    
private:
    GeometryUtility() {}
    ~GeometryUtility() {}
};


#endif // USE_BOOST_FILES_H