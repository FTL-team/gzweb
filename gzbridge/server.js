#!/usr/bin/env node

"use strict"

const WebSocketServer = require('websocket').server;
const http = require('http');
const fs = require('fs');
const path = require('path');
const gzbridge = require('./build/Debug/gzbridge');

/**
 * Path from where the static site is served
 */
const staticBasePath = './../http/client';

/**
 * Port to serve from, defaults to 8080
 */
const port = process.argv[2] || 7777;

/**
 * Array of websocket connections currently active, if it is empty, there are no
 * clients connected.
 */
let connections = [];

/**
 * Holds the message containing all material scripts in case there is no
 * gzserver connected
 */
let materialScriptsMessage = {};

/**
 * Whether currently connected to a gzserver
 */
let isConnected = false;

let gzNode = new gzbridge.GZNode();
if (gzNode.getIsGzServerConnected()) {
  gzNode.loadMaterialScripts(staticBasePath + '/assets');
  gzNode.setPoseMsgFilterMinimumAge(0.02);
  gzNode.setPoseMsgFilterMinimumDistanceSquared(0.00001);
  gzNode.setPoseMsgFilterMinimumQuaternionSquared(0.00001);

  console.log('--------------------------------------------------------------');
  console.log('Gazebo transport node connected to gzserver.');
  console.log('Pose message filter parameters between successive messages: ');
  console.log('  minimum seconds: ' +
    gzNode.getPoseMsgFilterMinimumAge());
  console.log('  minimum XYZ distance squared: ' +
    gzNode.getPoseMsgFilterMinimumDistanceSquared());
  console.log('  minimum Quartenion distance squared:'
    + ' ' + gzNode.getPoseMsgFilterMinimumQuaternionSquared());
  console.log('--------------------------------------------------------------');
}
else {
  materialScriptsMessage =
    gzNode.getMaterialScriptsMessage(staticBasePath + '/assets');
}

/**
 * Callback to serve static files
 * @param req Request
 * @param res Response
 */
let staticServe = function (req, res) {

  // CORS
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Request-Method', '*');
  res.setHeader('Access-Control-Allow-Methods', 'OPTIONS, GET');
  res.setHeader('Access-Control-Allow-Headers', '*');

  let fileLoc = path.resolve(staticBasePath);
  console.log(fileLoc);

  let urlParsed = req.url.split('?');
  req.url = urlParsed[0];
  let hr = urlParsed[1];
  if (!hr) hr = '';

  if (req.url === '/')
    req.url = '/index.html';

  if (req.url.indexOf('root/') != -1) {
    fileLoc = req.url.split('root/')[1];
  } else if (req.url.indexOf('file/') != -1) {
    fileLoc = gzNode.resolveFile('file://' + req.url.split('file/')[1]);
  } else if (req.url.indexOf('model/') != -1) {
    fileLoc = gzNode.resolveFile('model://' + req.url.split('model/')[1]);
  } else {
    fileLoc = path.join(fileLoc, req.url);
  }

  // gzNode.getMaterialScriptsMessage()


  res.statusCode = 200;

  console.log("FILE", req.url, fileLoc)
  let filesToSend = [fileLoc];
  function sendFile() {
    if (filesToSend.length == 0) {
      return res.end();
    }


    let currentFile = filesToSend.pop();
    console.log("Sending file", currentFile)
    let stat;
    try {
      stat = fs.lstatSync(currentFile);
    } catch (e) {
      res.statusCode = 404;
      res.write('404: File Not Found!');
      return res.end();
    }

    console.log(currentFile, stat.isDirectory())
    if (stat.isDirectory()) {
      fs.readdir(currentFile, function (err, files) {
        files.forEach(function (file) {
          filesToSend.push(path.join(currentFile, file))
        })
        sendFile()
      })
    } else {
      if (currentFile.endsWith(hr)) {
        fs.readFile(currentFile, function (err, data) {
          if (err) {
            res.write('404: File Not Found!');
            return res.end();
          }

          res.write(data);
          sendFile();
        });
      } else {
        sendFile();
      }
    }
  }
  sendFile()
};

// HTTP server
let httpServer = http.createServer(staticServe);
httpServer.listen(port);

console.log(new Date() + " Static server listening on port: " + port);

// Websocket

// Start websocket server
let wsServer = new WebSocketServer({
  httpServer: httpServer,
  // You should not use autoAcceptConnections for production
  // applications, as it defeats all standard cross-origin protection
  // facilities built into the protocol and the browser.  You should
  // *always* verify the connection's origin and decide whether or not
  // to accept it.
  autoAcceptConnections: false
});

wsServer.on('request', function (request) {

  // Accept request
  let connection = request.accept(null, request.origin);

  // If gzserver is not connected just send material scripts and status
  console.log("AAAAAA", gzNode.getIsGzServerConnected())
  if (!gzNode.getIsGzServerConnected()) {
    // create error status and send it
    let statusMessage =
      '{"op":"publish","topic":"~/status","msg":{"status":"error"}}';
    connection.sendUTF(statusMessage);
    // send material scripts message
    connection.sendUTF(materialScriptsMessage);
    return;
  }

  connections.push(connection);

  if (!isConnected) {
    isConnected = true;
    gzNode.setConnected(isConnected);
  }

  console.log(new Date() + ' New connection accepted from: ' + request.origin +
    ' ' + connection.remoteAddress);

  // Handle messages received from client
  connection.on('message', function (message) {
    if (message.type === 'utf8') {
      console.log(new Date() + ' Received Message: ' + message.utf8Data +
        ' from ' + request.origin + ' ' + connection.remoteAddress);
      gzNode.request(message.utf8Data);
    }
    else if (message.type === 'binary') {
      console.log(new Date() + ' Received Binary Message of ' +
        message.binaryData.length + ' bytes from ' + request.origin + ' ' +
        connection.remoteAddress);
      connection.sendBytes(message.binaryData);
    }
  });

  connection.on('close', function (reasonCode, description) {
    console.log(new Date() + ' Peer ' + request.origin + ' ' +
      connection.remoteAddress + ' disconnected.');

    // remove connection from array
    let conIndex = connections.indexOf(connection);
    connections.splice(conIndex, 1);

    // if there is no connection notify server that there is no connected client
    if (connections.length === 0) {
      isConnected = false;
      gzNode.setConnected(isConnected);
    }
  });
});

// If not connected, periodically send messages
if (gzNode.getIsGzServerConnected()) {
  setInterval(update, 10);

  function update() {
    if (connections.length > 0) {
      let msgs = gzNode.getMessages();
      for (let i = 0; i < connections.length; ++i) {
        for (let j = 0; j < msgs.length; ++j) {
          connections[i].sendUTF(msgs[j]);
        }
      }
    }
  }
}
