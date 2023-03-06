GZ3D.LedsController = function () {
  this.leds = {};
};

GZ3D.LedsController.prototype.subscribe = function (webSocket) {
  // console.log('subscribed', webSocket)
  var ledTopic = new ROSLIB.Topic({
    ros: webSocket,
    name: '~/leds',
    messageType: 'led',
  });
  ledTopic.subscribe(this.updateLeds.bind(this));
};

GZ3D.LedsController.prototype.updateLeds = function (state) {
  var that = this;
  Object.keys(state).forEach(function (key) {
    var led = that.leds[key];
    var color = state[key].color;
    if(led) {
      led.children.forEach(function (child) {
        child.material.color.setRGB(color[0], color[1], color[2]);
      });
    }
  });
};

GZ3D.LedsController.prototype.reegisterLed = function (obj) {
  var id = obj.name.split('led_link_visual_')[1];
  this.leds[id] = obj;
};