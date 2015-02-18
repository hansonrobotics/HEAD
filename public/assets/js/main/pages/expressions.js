define(['jquery', './../lib/api', './../lib/ros', './../lib/utilities', './../lib/crosshair-slider',
    'vendor/jquery.knob'], function ($, api, ros, utilities) {
    var expressions = {
        config: {
            knob: {
                max: 55,
                maxShow: 11,
                fgColor: "#df691a",
                fgColorExtreme: "#df691a"
            }
        },
        init: function () {
            $(".dial").val(0);
            this.initExpressions();
            this.createButtons();
        },

        loadPage: function () {
            var blenderMessage, blinkMessage, treeMessage;

            //blenderMessage = new ROSLIB.Message({data: 'Dummy'});
            //api.topics.cmdBlender.publish(blenderMessage);

            api.blenderMode.disable();

            blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
            api.topics.cmdBllink.publish(blinkMessage);

            treeMessage = new ROSLIB.Message({data: 'btree_off'});
            api.topics.cmdTree.publish(treeMessage);

            $('.expression-button.active').removeClass('active');

            api.setExpression("happy", 0);
            api.pointHead();
        },

        initExpressions: function () {
            //Create crosshair
            expressions.buildCrosshair(
                $(".crosshairsl"), {
                    bgColor: "#485563",
                    fgColor: "#fff"
                }
            ).on("change", function (e, ui) {
                    $(".crosshairsl")
                        .crosshairsl("option", "xval", ui.xval)
                        .crosshairsl("option", "yval", ui.yval);
                });

            //Create dial
            $(".dial").knob({
                min: 0, max: expressions.config.knob.max,
                angleOffset: -125, angleArc: 275,
                width: 200, height: 200,
                thickness: 0.4,
                fgColor: expressions.config.knob.fgColor, bgColor: "#444",
                draw: function () { //Extended the draw function to add a ring around the dial.
                    this.g.lineWidth = 3;
                    this.cursorExt = 0.2;

                    //Override radius to fit the outer circle inside canvas
                    this.radius = this.xy - this.lineWidth * 2 / 3 - this.g.lineWidth;

                    this.g.beginPath();
                    this.g.strokeStyle = this.o.fgColor;
                    this.g.arc(this.xy, this.xy, this.radius + 1 + this.lineWidth * 2 / 3, 0, 2 * Math.PI, false);
                    this.g.stroke();
                    return true;
                },
                change: expressions.updateExpression,
                format: function (value) {
                    return Math.round(value / expressions.config.knob.max * expressions.config.knob.maxShow);
                }
            });
            //Keep the input fields inside dials from being edited directly.
            $("input.dial").css("pointer-events", "none");
        },
        createButtons: function () {
            api.expressionList(function (response) {
                var capitalize = function (str) {
                    return str.charAt(0).toUpperCase() + str.slice(1);
                };

                $.each(response.exprnames, function () {
                    var button = $('<button type="button" class="btn btn-default expression-button">' + capitalize(this) + '</button>')
                        .data('expression', this)
                        .click(function () {
                            $('.expression-button').removeClass('active');
                            $(this).addClass('active');
                            expressions.current_face = $(this).data('expression');
                            $(".knobName").text($(this).html());

                            expressions.updateExpression(expressions.config.knob.max);
                        });

                    $(".expression-buttons").append(button);
                });
            });
        },
        updateExpression: function (value) {
            if (!value)
                value = parseInt($(".dial").val()) / expressions.config.knob.maxShow * expressions.config.knob.max;

            $(".dial").val(value).trigger("change");

            var intensity = (Math.round(value / expressions.config.knob.max *
            expressions.config.knob.maxShow) < expressions.config.knob.maxShow)
                ? value / (expressions.config.knob.max * 1.2) : 1.0;

            api.setExpression(expressions.current_face, intensity);
        },
        buildCrosshair: function (element, options) {
            var yaw = ros.getMotorConfig("neck_base");
            var pitch = ros.getMotorConfig("neck_pitch");

            $(element).crosshairsl($.extend({}, {
                xmin: Math.floor(utilities.radToDeg(-1.57)),
                xmax: Math.ceil(utilities.radToDeg(1.57)),
                xval: Math.round(utilities.radToDeg(0)),
                ymin: Math.floor(utilities.radToDeg(-0.6)),
                ymax: Math.ceil(utilities.radToDeg(0.6)),
                yval: Math.round(utilities.radToDeg(0)),
                change: function (e, ui) {
                    api.pointHead({
                        yaw: utilities.degToRad(ui.xval),
                        pitch: utilities.degToRad(-ui.yval)
                    });
                }
            }, options));

            return element;
        }
    };

    return expressions;

});
