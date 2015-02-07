var http = require('http');
var body_parser = require('body-parser');
var multer = require('multer');
var express = require('express');

// The built-in "fs" module provides filesystem-related functionality
var fs = require('fs');

// The built-in "path" module provides filesystem path-related functionality
var path = require('path');

// The add-on "mime" module provides the ability to derive a MIME type based on a filename extension
var mime = require('mime');

var toJSON = require('shp2json');

var formidable = require('formidable');

var mm = require('../node_port/build/Release/MMSolver');

var solver = new mm.MMSolver();

// The cache object is where the cotents of cached files are stored
var cache = {};

var done = false;

var port = 3000;

var upload_type;

var app = express();
app.use(function(err, req, res, next) {
    console.error(err.stack);
    next(err);
});

app.use(body_parser.json()); // for parsing application/json
app.use(body_parser.urlencoded({ extended: true })); // for parsing application/x-www-form-urlencoded

app.use(express.static(path.join(__dirname, 'lib')));
app.use(express.static(path.join(__dirname, 'data')));
app.use(express.static(path.join(__dirname, 'uploads')));

/*Configure the multer.*/

app.use(multer({ dest: './uploads/',
		 rename: function (fieldname, filename) {
		     return fieldname;
		 },
		 onFileUploadStart: function (file) {
		     console.log(file.originalname + ' is starting ...')
		 },
		 onFileUploadComplete: function (file) {
		     console.log(file.fieldname + ' uploaded to  ' + file.path)
		     done=true;
		 }
	       }));

// Sending error response
function send404(response) {
    response.writeHead(404, {'Content-Type': 'text/plain'});
    response.write('Error 404: resource not found.');
    response.end();
}

// Sending file data
function sendFile(response, filePath, fileContents) {
    response.writeHead(
	200,
	{'Content-Type': mime.lookup(path.basename(filePath))}
    );
    response.end(fileContents);
}

// Serving static files
function serveStatic(response, cache, absPath) {
    // Check if file exits
    fs.exists(absPath, function(exists) {
	if (exists) {
	    // Read data from disk
	    fs.readFile(absPath, function(err, data) {
		if (err) {
		    send404(response);
		} else {
		    cache[absPath] = data;
		    sendFile(response, absPath, data);
		}

	    });
	} else {
	    // Send HTTP 404 response
	    send404(response);
	}
    });
}

app.get('/', function(req, res) {
    // Determine HTML file to be served by default
    filePath = 'public/index.html';

    // Get ablsolute file path
    var absPath = './' + filePath;

    // Serve the static file
    serveStatic(res, cache, absPath);
});

app.post('/mm/upload', function(req, res) {
    /*
      var form = new formidable.IncomingForm();
      form.parse(req, function(err, fields, files){
      if(err) 
      return res.end('error');

      console.log('received fields:'); 
      console.log(fields);
      console.log('received files:'); 
      console.log(files);
      res.end('thank-you');
      });
    */
    if(done==true){
	res.redirect('/');
    }
});


app.post('/mm/match', function(req, res) {
    var sparse_gps = "./uploads/sparse_gps.geojson";
    var density_gps = "./uploads/density_gps.geojson";
    var out = "./uploads/match.geojson";
    var preprocess_dir = "../data/prepare";
    
    solver.match(preprocess_dir, sparse_gps, out, density_gps);
    
    res.redirect('/');

});

app.post('/mm/reset', function(req, res) {
    fs.exists('./uploads/sparse_gps.geojson', function(exists) {
	if (exists) {
	    fs.unlinkSync('./uploads/sparse_gps.geojson');
	}
    });
    
    fs.exists('./uploads/density_gps.geojson', function(exists) {
	if (exists) {
	    fs.unlinkSync('./uploads/density_gps.geojson');
	}
    });

    fs.exists('./uploads/match.geojson', function(exists) {
	if (exists) {
	    fs.unlinkSync('./uploads/match.geojson');
	}
    });

    res.redirect('/');
});

app.listen(port, function(status) {
    console.log("server run in " + port + " port");
});






