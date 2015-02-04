RosUI.motors = {
    init: function () {
        RosUI.motors.initMotors();
        RosUI.motors.initEdit();
        //setInterval(RosUI.motors.updateSliders, 1000);
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
    addSlider: function (config, editable) {
        if (typeof editable == 'undefined') editable = false;

        var sliderBlock = $("#app-slider-template").clone();
        sliderBlock.removeAttr("id"); //Removing app-slider-template id
        config.element = sliderBlock; //Saving a reference to html element in config object

        var degMin = Math.ceil(RosUI.utilities.radToDeg(config.min));
        var degMax = Math.floor(RosUI.utilities.radToDeg(config.max));
        var degVal = Math.round(RosUI.utilities.radToDeg(config.default));

        //Fill header
        var motor_name = config.labelleft || config.name;
        sliderBlock.find(".app-slider-label-left").text(motor_name);
        sliderBlock.find(".app-motor-name").val(motor_name);
        sliderBlock.find(".app-slider-label-right").text(config.labelright || "");
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

        if (editable) {
            sliderBlock.addClass('app-editable-motor');

            $('.app-motor-topic-name', sliderBlock).html(config.topic);
            $('.app-motor-id', sliderBlock).html(config.motor_id);

            sliderBlock.find('.app-motors-set-min').click(function () {
                config.min = $('.app-slider', sliderBlock).slider("value");
                $('.app-slider-min-value', sliderBlock).html(config.min);
            });

            sliderBlock.find('.app-motors-set-max').click(function () {
                config.max = $('.app-slider', sliderBlock).slider("value");
                $('.app-slider-max-value', sliderBlock).html(config.max);
            });

            sliderBlock.find('.app-motors-set-default').click(function () {
                config.default = $('.app-slider', sliderBlock).slider("value");
            });
        }
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
        console.log(motorConf);
        console.log(motorConf);
        for (var i = 0; i < motorConf.length; i++) {
            if (motorConf[i].name != "neck_pitch" && motorConf[i].name != "neck_base")
                RosUI.motors.addSlider(motorConf[i]);
        }
    },
    initEdit: function() {
        var editButton = $('#app-edit-motors-button'),
            saveButton = $('#app-save-motors-button'),
            motorConfig = [];

        editButton.click(function() {
            $(this).hide();
            $(saveButton).show();

            $(".app-editable-motor.app-motors-show-on-edit, .app-editable-motor .app-motors-show-on-edit", $('#app-page-motors')).show();
            $(".app-editable-motor.app-motors-hide-on-edit, .app-editable-motor .app-motors-hide-on-edit", $('#app-page-motors')).hide();

            $('#app-motor-sliders').sortable({axis: "y"});
        });

        saveButton.click(function() {
            $(this).hide();
            $(editButton).show();

            RosUI.motors.saveMotorConfig(motorConfig);

            $(".app-editable-motor.app-motors-hide-on-edit, .app-editable-motor .app-motors-hide-on-edit", $('#app-page-motors')).show();
            $(".app-editable-motor.app-motors-show-on-edit, .app-editable-motor .app-motors-show-on-edit", $('#app-page-motors')).hide();

            if ($('#app-motor-sliders').hasClass('ui-sortable'))
                $('#app-motor-sliders').sortable("destroy");
        });

        RosUI.api.getMotorTopicNames(function(response) {
            response.topics = ['fake1'];

            if (typeof response.topics != 'undefined') {
                $.each(response.topics, function() {
                    for (var i = 0; i < 24; i++) {
                        var config = {
                            name: "",
                            topic: this,
                            motor_id: i,
                            min: -1.5707965,
                            max: 1.5707965,
                            default: 0
                        };

                        var duplicate = false;
                        $.each(RosUI.ros.config.motors, function () {
                            if (this.topic == config.topic && this.motor_id == config.motor_id)
                                duplicate = true;
                        });

                        if (! duplicate) {
                            RosUI.motors.addSlider(config, true);
                            motorConfig.push(config);

                            var slider = config.element;
                            $(slider).addClass("app-motors-show-on-edit");
                        }
                    }
                });
            }
        });
    },
    saveMotorConfig: function(newMotorsConfig) {
        var config = [];
        $('#app-motor-sliders .app-slider:not(#app-slider-template)').each(function() {
            var container = $(this).closest('.app-slider-container'),
                slider = this,
                found = false;

            if ($('.app-motor-name', container).val().trim() == "")
                return;

            $.each(RosUI.ros.config.motors, function() {
                if (this.element.get(0) == container.get(0)) {
                    config.push(jQuery.extend({}, this));
                    found = true;
                }
            });

            $.each(newMotorsConfig, function() {
                if (this.element.get(0) == container.get(0)) {
                    config.push(jQuery.extend({}, this));
                    found = true;
                }
            });

            if (found) {
                var i = config.length - 1;
                config[i].labelleft = $('.app-motor-name', container).val();
                delete config[i].element;
                delete config[i].isActive;
            }
        });

        $.ajax("/motors/update_config", {
            data: JSON.stringify(config),
            type: 'POST',
            dataType: "json",
            success: function(response) {
                if (response) {
                    RosUI.ros.loadMotorConfig(function() {
                        RosUI.motors.destroySliders();
                        RosUI.motors.initMotors();
                        RosUI.motors.initEdit();
                    })
                }
            }
        });
    },
    destroySliders: function() {
        $('#app-motor-sliders li:not(#app-slider-template)').hide().remove();
    }
};
