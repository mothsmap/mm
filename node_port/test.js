var mm = require('./build/Release/MMSolver');

var solver = new mm.MMSolver();

var node = "../data/raw/nodes.shp";
var edge = "../data/raw/edges.shp";
var history =  "../data/raw/history.shp";
var preprocess_dir = "../data/test";
var sparse_gps = "../data/test/cross_1m.shp";
var density_gps = "../data/test/cross.shp";

var sparse_gps2 = "../data/test/cross_1m.geojson";
var density_gps2 = "../data/test/cross.geojson";

var out = "../data/test/out.geojson";

var xmin = 12701618.12;
var ymin = 4523858.73;
var xmax = 12796422.31;
var ymax = 4626505.94;
var expand = 1000.0;
/*
solver.prepare(
    node, edge, history, preprocess_dir, xmin - expand, ymin - expand, xmax + expand, ymax + expand
);

solver.match(
    preprocess_dir, sparse_gps, out, density_gps
);
*/

solver.match(
    preprocess_dir, sparse_gps2, out, density_gps2
);

console.log("Done.");
