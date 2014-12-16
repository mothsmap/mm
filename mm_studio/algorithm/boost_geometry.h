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

struct envelope_visitor : public boost::static_visitor<BoostBox>
{
    BoostBox operator()(BoostPoint const& g) const { return bg::return_envelope<BoostBox>(g); }
    BoostBox operator()(BoostLineString const& g) const { return bg::return_envelope<BoostBox>(g); }
    BoostBox operator()(BoostPolygon const& g) const { return bg::return_envelope<BoostBox>(g); }
};

template<typename Point, typename Segment>
bool point_to_segment_org(Point const& point, Segment const& segment, Point& ptOut)
{
    bool bInSegment = true;
    
    try
    {
        typedef typename bg::point_type<Segment>::type point_type;
        point_type p[2] = {segment.first, segment.second};
        
        if (boost::geometry::equals(point, p[0]) ||
            boost::geometry::equals(point, p[1]))
        {
            boost::geometry::assign_point(ptOut, point);
            bInSegment = true;
            throw bInSegment;
        }
        
        point_type v(p[1]), w(point);
        
        boost::geometry::subtract_point(v, p[0]);
        boost::geometry::subtract_point(w, p[0]);
        
        typedef typename bg::select_calculation_type<Point, Segment, void>::type calculation_type;
        
        calculation_type const zero = calculation_type();
        calculation_type const c1 = boost::geometry::dot_product(w, v);
        if (c1 < zero)
        {
            bInSegment = false;
        }
        
        double const c2 = boost::geometry::dot_product(v, v);
        if (c2 < c1)
        {
            bInSegment = false;
        }
        
        calculation_type const b = c1 / c2;
        bg::model::point<double, 2, bg::cs::cartesian> projected(p[0]);
        
        boost::geometry::multiply_value(v, b);
        boost::geometry::add_point(projected, v);  
        boost::geometry::assign_point(ptOut, projected);  
    }  
    catch (bool)  
    {  
    }  
    
    return bInSegment;  
}

#endif // USE_BOOST_FILES_H