var express = require('express'),
    cluster = require('cluster'),
    http = require('http'),
    Analyze = require('./build/Release/GeoAnalyze');


// set max connection
http.globalAgent.maxSockets = 40000;

// Use all the cpu
var num_cpus = require('os').cpus().length;
if (cluster.isMaster) {
    for (var i = 0; i < num_cpus; i++) {
       console.log("fork ...");
        cluster.fork();
    }

    // Listen for dying processes
    cluster.on('exit', function(worker, code, signal) {
        console.log('worker ' + worker.process.pid + ' died');
        cluster.fork();
    });

    return;
}


var app = express();

app.get('/buffer', function(req, res) {
    // create GEOS context
    var context = new Analyze.GeosContext();
    context.create();

    var analyze = new Analyze.GeoAnalyze(context);


    var input_geometry = "POINT(0 0)";
    var width = 10;

    var output_geometry = analyze.bufferSync(input_geometry, width);
    res.type('text/plain');
    res.end(output_geometry);
    // analyze.buffer(input_geometry, width, function(output_geometry) {
    //     context.destroy();

    //     res.type('text/plain');
    //     res.end(output_geometry);
    // });
});

app.listen(3000);

