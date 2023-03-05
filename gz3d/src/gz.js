var GZ3D = GZ3D || {
  REVISION: '1',
  pathFromURI: function (uri) {
    var uriType = uri.substring(0, uri.indexOf('://'));
    if (uriType === 'file') {
      uri = 'file/' + uri.substring(uri.indexOf('://') + 3);
    } else if (uriType === 'model') {
      uri = 'model/' + uri.substring(uri.indexOf('://') + 3);
    }
    else if (uri.length > 0 && uri[0] === '/') {
      uri = 'root/' + uri;
    }
    
    return location.href + '/' + uri;
  }
};

var globalEmitter = new EventEmitter2({ verboseMemoryLeak: true });

// Assuming all mobile devices are touch devices.
var isTouchDevice = /Mobi/.test(navigator.userAgent);