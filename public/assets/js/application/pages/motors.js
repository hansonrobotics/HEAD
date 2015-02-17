define(['jquery', './../lib/api', './../lib/ros', './../lib/utilities'], function ($, api, ros, utilities) {
    var motors = {
        init: function () {
            motors.initMotors();
            motors.initEdit();
            //setInterval(motors.updateSliders, 1000);
        },
        loadPage: function () {
            var blenderMessage, blinkMessage, treeMessage;

            blenderMessage = new ROSLIB.Message({data: 'Dummy'});
            api.topics.cmdBlender.publish(blenderMessage);

            blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
            api.topics.cmdBllink.publish(blinkMessage);

            treeMessage = new ROSLIB.Message({data: 'btree_off'});
            api.topics.cmdTree.publish(treeMessage);

            api.setExpression("happy", 0);
            api.pointHead({yaw: 0, pitch: 0, roll: 0});

        },
        addSlider: function (config) {
            var sliderBlock = $("#app-slider-template").clone();
            sliderBlock.removeAttr("id"); //Removing app-slider-template id
            config.element = sliderBlock; //Saving a reference to html element in config object

            var degMin = Math.ceil(utilities.radToDeg(config.min));
            var degMax = Math.floor(utilities.radToDeg(config.max));
            var degVal = Math.round(utilities.radToDeg(config.default));

            //Fill header
            var motor_name = config.labelleft;
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
                            api.pointHead({roll: utilities.degToRad(ui.value)});
                        }
                    } else {
                        return function (e, ui) {
                            slVal.text(ui.value);
                            api.sendMotorCommand(config, utilities.degToRad(ui.value));
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

            if (typeof config.editable != 'undefined' && config.editable == true) {
                sliderBlock.addClass('app-editable-motor');

                $('.app-motor-topic-name', sliderBlock).html(config.topic);
                $('.app-motor-id', sliderBlock).html(config.name);

                sliderBlock.find('.app-motors-set-min').click(function () {
                    var deg = $('.app-slider', sliderBlock).slider("value");
                    config.min = utilities.degToRad(deg);
                    $('.app-slider-min-value', sliderBlock).html(deg);
                });

                sliderBlock.find('.app-motors-set-max').click(function () {
                    var deg = $('.app-slider', sliderBlock).slider("value");
                    config.max = utilities.degToRad(deg);
                    $('.app-slider-max-value', sliderBlock).html(deg);
                });

                sliderBlock.find('.app-motors-set-default').click(function () {
                    config.default = utilities.degToRad($('.app-slider', sliderBlock).slider("value"));
                });
            }
        },
        updateSliders: function () {
            var motorConf = api.config.motors;
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
            var motorConf = api.config.motors;

            for (var i = 0; i < motorConf.length; i++) {
                if (motorConf[i].name != "neck_pitch" && motorConf[i].name != "neck_base")
                    motors.addSlider(motorConf[i]);
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

                $('.app-motor-drag-handle').show();

                $('#app-motor-sliders').sortable({
                    axis: "y",
                    handle: ".app-motor-drag-handle",
                    placeholder: "ui-state-highlight"
                });
            });

            saveButton.click(function() {
                $(this).hide();

                motors.saveMotorConfig(motorConfig);

                $(".app-editable-motor.app-motors-hide-on-edit, .app-editable-motor .app-motors-hide-on-edit", $('#app-page-motors')).show();
                $(".app-editable-motor.app-motors-show-on-edit, .app-editable-motor .app-motors-show-on-edit", $('#app-page-motors')).hide();

                $('.app-motor-drag-handle').hide();

                if ($('#app-motor-sliders').hasClass('ui-sortable'))
                    $('#app-motor-sliders').sortable("destroy");
            });

            api.getPololuMotorTopics(function(topics) {
                $.each(topics, function() {
                    for (var i = 0; i < 24; i++) {
                        var config = {
                            name: i,
                            topic: this,
                            min: - Math.PI / 2,
                            max: Math.PI / 2,
                            default: 0,
                            editable: true
                        };

                        var duplicate = false;
                        $.each(api.config.motors, function () {
                            if (this.topic == config.topic && this.name == config.name) {
                                duplicate = true;
                                config = this;
                            }
                        });

                        if (! duplicate) {
                            motors.addSlider(config);
                            $(config.element).addClass("app-motors-show-on-edit");
                        }

                        motorConfig.push(config);
                    }

                });

                $(editButton).fadeIn();
            });
        },
        saveMotorConfig: function(newMotorsConfig) {
            var config = [];
            $('#app-motor-sliders .app-slider-container:not(#app-slider-template)').each(function() {
                var container = this,
                    slider = $('.app-slider', this);

                if ($('.app-motor-name', container).val().trim() == "")
                    return;

                $.each(newMotorsConfig.concat(api.config.motors), function() {
                    if (this.element.get(0) == container) {
                        var i = config.length;

                        config.push($.extend({}, this));
                        config[i].labelleft = $('.app-motor-name', container).val();

                        delete config[i].element;
                        delete config[i].isActive;

                        // break each
                        return false;
                    }
                });
            });

            $.ajax("/motors/update_config", {
                data: JSON.stringify(config),
                type: 'POST',
                dataType: "json",
                success: function(response) {
                    if (response) {
                        ros.loadMotorConfig(function() {
                            motors.destroySliders();
                            motors.initMotors();
                            motors.initEdit();
                        })
                    }
                }
            });
        },
        destroySliders: function() {
            $('#app-motor-sliders li:not(#app-slider-template)').hide().remove();
        }
    };

    return motors;
});
