var RoboInterface = {

  $: $({}), //Event proxy. Events are triggered and bound to this object.

  sendMotorCmd: function(confEntry, angle, speed, acc) {
    var cmd = new ROSLIB.Message({
      id: confEntry.motorid,
      angle: Math.min(Math.max(angle, confEntry.min), confEntry.max),
      speed: speed || confEntry.speed,
      acceleration: acc || confEntry.acceleration
    });
    this.motorCmdTopic.publish(cmd);
  },

  sendDefaultMotorCmds: function() {
    for (var i = 0; i < this.motorConf.length; i++) {
      this.sendMotorCmd(this.motorConf[i], this.motorConf[i].default);
    };
  },

  getValidFaceExprs: function(callback) {
    this.validFaceExprsClient.callService(
      new ROSLIB.ServiceRequest(),
      callback
    );
  },

  makeFaceExpr: function(faceStr, intensity) {
    this.makeFaceExprTopic.publish(
      new ROSLIB.Message({
        exprname:faceStr,
        intensity:intensity
      })
    );
  },

  //Loads the config file and connects to ROS
  connect: function(address) {

    function connectROS() {
      //Connect to rosbridge
      ros = new ROSLIB.Ros({
        url: address
      }).on("connection", function(e){
        RoboInterface.$.trigger("connection");
        RoboInterface.sendDefaultMotorCmds();
      }).on("error", function(e){
        RoboInterface.$.trigger("error");
      });

      //Publish topic
      RoboInterface.motorCmdTopic = new ROSLIB.Topic({
        ros:ros,
        name:'/cmd_pololu',
        messageType:'ros_pololu_servo/servo_pololu'
      });
      RoboInterface.makeFaceExprTopic = new ROSLIB.Topic({
        ros:ros,
        name:'/make_face_expr',
        messageType:'basic_head_api/MakeFaceExpr'
      })

      //Subscribe to topic
      RoboInterface.motorCmdTopic.subscribe(function(msg) {
        RoboInterface.$.trigger("onMotorCmd", {
          msg: msg,
          confEntry: getConfFromID(msg.id)
        });
      });

      //Set up services
      RoboInterface.validFaceExprsClient = new ROSLIB.Service({
        ros:ros,
        name:'/valid_face_exprs',
        serviceType:'basic_head_api/ValidFaceExprs'
      });
    };
    this.$.on("configload", connectROS);

    $.ajax({
        url: "config.yaml",
        dataType: "text",
        success: function(data) {
          RoboInterface.motorConf = jsyaml.load(data);
          RoboInterface.$.trigger("configload");
        }
    });

    return this;
  },
};

//Utility function
var getConfFromID = (function() {
  var motorID2Conf = {};

  RoboInterface.$.on("configload", function() {
    var motorConf = RoboInterface.motorConf;
    for (var i = 0; i < motorConf.length; i++) {
      motorID2Conf[motorConf[i].motorid] = motorConf[i];
    };
  });

  return function (id) {
    return motorID2Conf[id];
  };
})()