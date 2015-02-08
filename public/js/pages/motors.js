RosUI.motors = {
    init: function () {
        RosUI.motors.initMotors();
        setInterval(RosUI.motors.updateSliders, 1000);
    },
    loadPage: function () {
        var blenderMessage, blinkMessage, treeMessage;

        RosUI.api.blenderMode.disable();

        blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
        RosUI.ros.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_off'});
        RosUI.ros.topics.cmdTree.publish(treeMessage);

        RosUI.motors.initMotors();
        RosUI.api.pointHead({yaw: 0, pitch: 0, roll: 0});

    },
    addSlider: function (config) {
        var sliderBlock = $("#sliderTemplate").clone();
        sliderBlock.removeAttr("id"); //Removing sliderTemplate id
        config.element = sliderBlock; //Saving a reference to html element in config object
        var degMin = Math.ceil(RosUI.utilities.radToDeg(config.min));
        var degMax = Math.floor(RosUI.utilities.radToDeg(config.max));
        var degVal = Math.round(RosUI.utilities.radToDeg(config.default));

        //Fill header
        sliderBlock.find(".slNameL").text(config.labelleft || config.name);
        sliderBlock.find(".slNameR").text(config.labelright || "");
        sliderBlock.find(".slMin").text(degMin);
        sliderBlock.find(".slMax").text(degMax);
        sliderBlock.find(".slVal").text(degVal);

        //Create slider
        var slVal = sliderBlock.find(".slVal");
        var slider = sliderBlock.find(".slider");
        slider.slider({
            range: "min",
            value: degVal,
            min: degMin,
            max: degMax,
            slide: (function () {
                if (config.name == "neck_roll") {
                    return function (e, ui) {
                        slVal.text(ui.value);
                        RosUI.api.pointHead({roll: RosUI.utilities.degToRad(ui.value)});
                    }
                } else {
                    return function (e, ui) {
                        slVal.text(ui.value);
                        RosUI.api.sendMotorCommand(config, RosUI.utilities.degToRad(ui.value));
                    }
                }
            })(),
            start: function (e, ui) {
                config.isActive = true;
            },
            stop: function (e, ui) {
                config.isActive = false;
            }
        });

        sliderBlock.removeClass("hidden");
        sliderBlock.appendTo("#sliderStack");
    },
    updateSliders: function () {
        var motorConf = RosUI.ros.config.motors;
        for (var i = 0; i < motorConf.length; i++) {
            if (motorConf[i].name != "neck_pitch" && motorConf[i].name != "neck_base")
                if (motorConf[i].newVal != undefined) {
                    motorConf[i].element.find(".slider").slider("value", motorConf[i].newVal);
                    motorConf[i].element.find(".slVal").text(motorConf[i].newVal);
                    delete motorConf[i].newVal;
                }
        }
    },
    initMotors: function () {
        var motorConf = RosUI.ros.config.motors;
        for (var i = 0; i < motorConf.length; i++) {
            if (motorConf[i].name != "neck_pitch" && motorConf[i].name != "neck_base")
                RosUI.motors.addSlider(motorConf[i]);
        }
    }
};
