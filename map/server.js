var http = require('http');
var body_parser = require('body-parser');
var express = require('express');

// The built-in "fs" module provides filesystem-related functionality
var fs = require('fs');

// The built-in "path" module provides filesystem path-related functionality
var path = require('path');

// The add-on "mime" module provides the ability to derive a MIME type based on a filename extension
var mime = require('mime');

var toJSON = require('shp2json');

var formidable = require('formidable');

// The cache object is where the cotents of cached files are stored
var cache = {};

var port = 3000;
var app = express();
app.use(function(err, req, res, next) {
    console.error(err.stack);
    next(err);
});

app.use(body_parser.json()); // for parsing application/json
app.use(body_parser.urlencoded({ extended: true })); // for parsing application/x-www-form-urlencoded

app.use(express.static(path.join(__dirname, 'lib')));
app.use(express.static(path.join(__dirname, 'data')));

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

app.post('/convert', function(req, res) {
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
});

app.listen(port, function(status) {
    console.log("server run in " + port + " port");
});






