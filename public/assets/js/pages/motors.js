RosUI.motors = {
    init: function () {
        RosUI.motors.initMotors();
        RosUI.motors.initEdit();
        setInterval(RosUI.motors.updateSliders, 1000);
    },
    loadPage: function () {
        var blenderMessage, blinkMessage, treeMessage;

        blenderMessage = new ROSLIB.Message({data: 'Dummy'});
        RosUI.ros.topics.cmdBlender.publish(blenderMessage);

        blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
        RosUI.ros.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_off'});
        RosUI.ros.topics.cmdTree.publish(treeMessage);

        RosUI.api.setExpression("happy", 0);
        RosUI.api.pointHead({yaw: 0, pitch: 0, roll: 0});

    },
    addSlider: function (config) {
        var sliderBlock = $("#app-slider-template").clone();
        sliderBlock.removeAttr("id"); //Removing app-slider-template id
        config.element = sliderBlock; //Saving a reference to html element in config object
        var degMin = Math.ceil(RosUI.utilities.radToDeg(config.min));
        var degMax = Math.floor(RosUI.utilities.radToDeg(config.max));
        var degVal = Math.round(RosUI.utilities.radToDeg(config.default));

        //Fill header
        sliderBlock.find(".app-slider-label-left").text(config.labelleft || config.name);
        sliderBlock.find(".slNameR").text(config.labelright || "");
        sliderBlock.find(".app-slider-min-value").text(degMin);
        sliderBlock.find(".app-slider-max-value").text(degMax);
        sliderBlock.find(".app-slider-value").text(degVal);

        //Create slider
        var slVal = sliderBlock.find(".app-slider-value");
        var slider = sliderBlock.find(".app-slider");
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
        sliderBlock.appendTo("#app-motor-sliders");
    },
    updateSliders: function () {
        var motorConf = RosUI.ros.config.motors;
        for (var i = 0; i < motorConf.length; i++) {
            if (motorConf[i].name != "neck_pitch" && motorConf[i].name != "neck_base")
                if (motorConf[i].newVal != undefined) {
                    motorConf[i].element.find(".app-slider").slider("value", motorConf[i].newVal);
                    motorConf[i].element.find(".app-slider-value").text(motorConf[i].newVal);
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
    },
    initEdit: function() {
        var editButton = $('#app-edit-motors-button'),
            saveButton = $('#app-save-motors-button');

        editButton.click(function() {
            $(this).hide();
            $(saveButton).show();

            $('#app-motor-sliders').sortable({axis: "y"});
        });

        saveButton.click(function() {
            $(this).hide();
            $(editButton).show();

            if ($('#app-motor-sliders').hasClass('ui-sortable'))
                $('#app-motor-sliders').sortable("destroy");
        }).click();

        RosUI.api.getMotorTopicNames(function(response) {
            response.topics = ['fake1', 'fake2'];

            if (typeof response.topics != 'undefined') {
                $.each(response.topics, function() {

                });
            }
        });
    }
};
