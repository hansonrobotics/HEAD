function radToDeg(val) {
    return val * 180 / Math.PI;
}
function degToRad(val) {
    return val * Math.PI / 180;
}

function addSliderBlock(confEntry) {
    var sliderBlock = $("#sliderTemplate").clone();
    sliderBlock.removeAttr("id"); //Removing sliderTemplate id
    confEntry.element = sliderBlock //Saving a reference to html element in config object
    var degMin = Math.ceil(radToDeg(confEntry.min));
    var degMax = Math.floor(radToDeg(confEntry.max));
    var degVal = Math.round(radToDeg(confEntry.default));

    //Fill header
    sliderBlock.find(".slNameL").text(confEntry.labelleft || confEntry.name);
    sliderBlock.find(".slNameR").text(confEntry.labelright);
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
            if (confEntry.name == "neck_roll") {
                return function (e, ui) {
                    slVal.text(ui.value);
                    RoboInterface.pointHead({roll: degToRad(ui.value)});
                }
            } else {
                return function (e, ui) {
                    slVal.text(ui.value);
                    RoboInterface.sendMotorCmd(confEntry, degToRad(ui.value));
                }
            }
        })(),
        start: function (e, ui) {
            confEntry.isActive = true;
        },
        stop: function (e, ui) {
            confEntry.isActive = false;
        }
    });

    sliderBlock.removeClass("hidden");
    sliderBlock.appendTo("#sliderStack");
}

function updateUI() {
    motorConf = RoboInterface.motorConf;
    for (var i = 0; i < motorConf.length; i++) {
        if (motorConf[i].newVal != undefined) {
            motorConf[i].element.find(".slider").slider("value", motorConf[i].newVal);
            motorConf[i].element.find(".slVal").text(motorConf[i].newVal);
            delete motorConf[i].newVal;
        }
    }
}

//Create UI that requires a loaded config file.
function initMotors() {
    //Create sliders
    motorConf = RoboInterface.motorConf;
    for (var i = 0; i < motorConf.length; i++) {
        addSliderBlock(motorConf[i]);
    }

    //Update UI according to messages
    RoboInterface.$.on("onMotorCmd", function (e, msgObj) {
        var msg = msgObj.msg;
        var confEntry = msgObj.confEntry;

        //Ignore neck motors (if they have no motorid defined).
        if (confEntry == undefined)
            return;

        if (confEntry.isActive)
            return;
        var degAngle;

        if (msgObj.topic.messageType == 'std_msgs/Float64') {
            degAngle = Math.round(radToDeg(msg.data));
        } else {
            degAngle = Math.round(radToDeg(msg.angle));
        }
        //confEntry.element.find(".slider").slider("value", degAngle);
        //confEntry.element.find(".slVal").text(degAngle);
        confEntry.newVal = degAngle;
    });

    //Create crosshair
    CommonUI.buildCrosshairSlider($(".crosshairsl"));
}

$(function () {
    RosUI.ros.$.on("connection", function () {
        initMotors();
        setInterval(updateUI, 1000);
    });
});