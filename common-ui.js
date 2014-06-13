var CommonUI = new function() {

  function capitalize(str) {
    return str.charAt(0).toUpperCase() + str.slice(1);
  }

  //Look up motor config entry by its name.
  function getConf(name) {
    var motorConf = RoboInterface.motorConf;
    for (var i = 0; i < motorConf.length; i++) {
      if (motorConf[i].name == name)
        return motorConf[i];
    };
    return;
  }

  this.buildExpressionButtons = function(container) {
    function addBtn(btnObj) {
      btn = $('<button type="button" class="btn btn-warning">'+btnObj['label']+'</button>');
      btn.click(function(){
        container.trigger("exprbtnclick", btnObj);
      });
      container.append(btn);
      return btn;
    }

    //Look up valid expression names through ROS.
    var motorConf = RoboInterface.motorConf;
    RoboInterface.getValidFaceExprs(function(response) {
      var names = response.exprnames;
      var btnObjs = [];
      for (var i = 0; i < names.length; i++) {
        btnObjs[i] = {
          name: names[i],
          label: (names[i] == "eureka") ? "Eureka!" : capitalize(names[i])
        };
        btnObjs[i]['element'] = addBtn(btnObjs[i]);
      };
      container.trigger("success", {btnObjs:btnObjs}); //Can't send an array without an object wrapper
    });
  }

  this.buildCrosshairSlider = function(element, options) {
    var yawConf = getConf("neck_base");
    var pitchConf = getConf("neck_pitch");
    element.crosshairsl($.extend({}, {
      xmin: Math.floor(radToDeg(yawConf.min)),
      xmax: Math.ceil(radToDeg(yawConf.max)),
      xval: Math.round(radToDeg(yawConf.default)),
      ymin: Math.floor(radToDeg(pitchConf.min)),
      ymax: Math.ceil(radToDeg(pitchConf.max)),
      yval: Math.round(radToDeg(pitchConf.default)),
      change: function(e, ui) {
        RoboInterface.sendMotorCmd(yawConf, degToRad(ui.xval));
        RoboInterface.sendMotorCmd(pitchConf, degToRad(ui.yval));
      }
    }, options));
    return element;
  }

};