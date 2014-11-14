var CommonUI = new function () {

    function capitalize(str) {
        return str.charAt(0).toUpperCase() + str.slice(1);
    }

    //Look up motor config entry by its name.
    function getConf(name) {
        var motorConf = RoboInterface.motorConf;
        for (var i = 0; i < motorConf.length; i++) {
            if (motorConf[i].name == name)
                return motorConf[i];
        }
    }

    this.buildExpressionButtons = function (container, classStr) {
        function addBtn(btnObj) {
            btn = $('<button type="button" class="btn btn-default expression-button">' + btnObj['label'] + '</button>');
            btn.addClass(classStr).addClass(btnObj.name);

            $(container).each(function() {
                var specific_container = this,
                    specific_button = $(btn).clone();

                specific_button.click(function () {
                    $(specific_container).trigger("exprbtnclick", btnObj);
                });

                $(specific_container).append(specific_button);
            });

            return btn;
        }

        //Look up valid expression names through ROS.
        var motorConf = RoboInterface.motorConf;
        RoboInterface.getValidFaceExprs(function (response) {
            var names = response.exprnames;
            var btnObjs = [];
            for (var i = 0; i < names.length; i++) {
                btnObjs[i] = {
                    name: names[i],
                    label: (names[i] == "eureka") ? "Eureka!" : capitalize(names[i])
                };
                btnObjs[i]['element'] = addBtn(btnObjs[i]);
            }

            container.trigger("success", {btnObjs: btnObjs}); //Can't send an array without an object wrapper
        });
    };

    this.buildCrosshairSlider = function (element, options) {
        var yawConf = getConf("neck_base");
        var pitchConf = getConf("neck_pitch");
        element.crosshairsl($.extend({}, {
            xmin: Math.floor(radToDeg(yawConf.min)),
            xmax: Math.ceil(radToDeg(yawConf.max)),
            xval: Math.round(radToDeg(yawConf.default)),
            //Minus signs and min-max swapped to invert the y axis for pitch to point upwards.
            ymin: Math.floor(radToDeg(-pitchConf.max)),
            ymax: Math.ceil(radToDeg(-pitchConf.min)),
            yval: Math.round(radToDeg(-pitchConf.default)),
            change: function (e, ui) {
                RoboInterface.pointHead({
                    yaw: degToRad(ui.xval),
                    pitch: degToRad(-ui.yval)
                });
            }
        }, options));
        return element;
    }
};

function websocketAddress() {
    return "ws://192.168.0.69d:9090";

    if (window.location.protocol != "https:") {
        return "ws://" + document.domain + ":9090";
    } else {
        return "wss://" + document.domain + ":9092";
    }
}