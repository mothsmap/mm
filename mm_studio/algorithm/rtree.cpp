#include "rtree.h"
#include "ogrsf_frmts.h"

	RTree::RTree() {
        OGRRegisterAll();
	}

	RTree::~RTree(void) {
	}

    bool RTree::AddNodes(std::string map_dir) {
#if PRINT_INFO
        std::cout << "Add nodes to Rtree...\n";
#endif
        nodes_.clear();
        node_tree_.clear();

        std::string file = map_dir + "/mm/nodes.shp";
        OGRDataSource* data_source = OGRSFDriverRegistrar::Open(file.c_str(), FALSE);
        if (data_source == NULL) {
            return false;
        }

        OGRLayer* layer = data_source->GetLayer(0);
        OGRFeature* feature = layer->GetNextFeature();
        int feature_count = 0;
        while (feature != NULL) {
            // feature geometry
            OGRGeometry* poGeometry = feature->GetGeometryRef();
            if (poGeometry == NULL || wkbFlatten(poGeometry->getGeometryType()) != wkbPoint) {
                OGRFeature::DestroyFeature (feature);
                feature = layer->GetNextFeature();
                continue;
            }

            // feature
            OGRPoint *point = (OGRPoint*) poGeometry;
            BoostPoint boost_point(point->getX(), point->getY());

            nodes_.insert(std::make_pair(feature_count, boost_point));

            // RTree
            BoostBox b = bg::return_envelope<BoostBox>(boost_point);
            node_tree_.insert(std::make_pair(b, feature_count));
            feature_count++;

            OGRFeature::DestroyFeature (feature);
            feature = layer->GetNextFeature();
        }
        return true;
    }

    bool RTree::AddEdges(std::string map_dir) {
#if PRINT_INFO
        std::cout << "Add edges to Rtree...\n";
#endif
        edges_.clear();
        edge_tree_.clear();

        std::string file = map_dir + "/mm/edges.shp";
        OGRDataSource* data_source = OGRSFDriverRegistrar::Open(file.c_str(), FALSE);
        if (data_source == NULL) {
            return false;
        }

        OGRLayer* layer = data_source->GetLayer(0);
        OGRFeature* feature = layer->GetNextFeature();
        int feature_count = 0;
        while (feature != NULL) {
            // feature geometry
            OGRGeometry* poGeometry = feature->GetGeometryRef();
            if (poGeometry == NULL || wkbFlatten(poGeometry->getGeometryType()) != wkbLineString) {
                OGRFeature::DestroyFeature (feature);
                feature = layer->GetNextFeature();
                continue;
            }
            // info
            RoadInfo info;
            info.oneway_ = feature->GetFieldAsInteger("oneway");
            info.travel_counts_ = 0;
            //info.name_ = feature->GetFieldAsString("name");
            info_.insert(std::make_pair(feature_count, info));

            // feature
            OGRLineString *line_string = (OGRLineString*) poGeometry;
            BoostLineString boost_line;
            for (int i = 0; i < line_string->getNumPoints(); ++i) {
                OGRPoint point;
                line_string->getPoint(i, &point);

                boost_line.push_back(BoostPoint(point.getX(), point.getY()));
            }
            edges_.insert(std::make_pair(feature_count, boost_line));

            // RTree
            BoostBox b = bg::return_envelope<BoostBox>(boost_line);
            edge_tree_.insert(std::make_pair(b, feature_count));
            feature_count++;

            OGRFeature::DestroyFeature (feature);
            feature = layer->GetNextFeature();
        }
        return true;
    }

	bool RTree::Build(std::string map_dir) {
        return (AddNodes(map_dir) && AddEdges(map_dir));
	}

	std::vector<Value> RTree::Query(QueryType type, double xmin, double ymin, double xmax, double ymax) {
		BoostBox query_box(BoostPoint(xmin, ymin), BoostPoint(xmax, ymax));
		std::vector<Value> results;
        if (type == EDGE)
            edge_tree_.query(bgi::intersects(query_box), std::back_inserter(results));
        else
            node_tree_.query(bgi::intersects(query_box), std::back_inserter(results));

		return results;
	}

	std::vector<Value> RTree::Query(QueryType type, BoostPoint pt, int elements) {
		std::vector<Value> results;
        if (type == EDGE)
            edge_tree_.query(bgi::nearest(pt, elements), std::back_inserter(results));
        else
            node_tree_.query(bgi::nearest(pt, elements), std::back_inserter(results));

		return results;
	}

void RTree::TravelEdge(int id) {
    info_[id].travel_counts_++;
  //  std::cout << "Edge " << id << " travel count = " << info_[id].travel_counts_ << std::endl;
}
