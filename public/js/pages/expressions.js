RosUI.expressions = {
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
        //RosUI.ros.topics.cmdBlender.publish(blenderMessage);

        RosUI.api.blenderMode.disable();

        blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
        RosUI.ros.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_off'});
        RosUI.ros.topics.cmdTree.publish(treeMessage);

        $('.expression-button.active').removeClass('active');

        RosUI.motors.initMotors();
        RosUI.api.pointHead();
    },

    initExpressions: function () {
        //Create crosshair
        RosUI.expressions.buildCrosshair(
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
            min: 0, max: RosUI.expressions.config.knob.max,
            angleOffset: -125, angleArc: 275,
            width: 200, height: 200,
            thickness: 0.4,
            fgColor: RosUI.expressions.config.knob.fgColor, bgColor: "#444",
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
            change: RosUI.expressions.updateExpression,
            format: function (value) {
                return Math.round(value / RosUI.expressions.config.knob.max * RosUI.expressions.config.knob.maxShow);
            }
        });
        //Keep the input fields inside dials from being edited directly.
        $("input.dial").css("pointer-events", "none");
    },
    createButtons: function () {
        console.log("Creating Buttons")
        RosUI.api.expressionList(function (response) {
            var capitalize = function (str) {
                return str.charAt(0).toUpperCase() + str.slice(1);
            };

            $.each(response.exprnames, function () {
                var button = $('<button type="button" class="btn btn-default expression-button">' + capitalize(this) + '</button>')
                    .data('expression', this)
                    .click(function () {
                        $('.expression-button').removeClass('active');
                        $(this).addClass('active');
                        RosUI.expressions.current_face = $(this).data('expression');
                        $(".knobName").text($(this).html());

                        RosUI.expressions.updateExpression(RosUI.expressions.config.knob.max);
                    });

                $(".expression-buttons").append(button);
            });
        });
    },
    updateExpression: function (value) {
        if (!value)
            value = parseInt($(".dial").val()) / RosUI.expressions.config.knob.maxShow * RosUI.expressions.config.knob.max;

        $(".dial").val(value).trigger("change");

        var intensity = (Math.round(value / RosUI.expressions.config.knob.max *
        RosUI.expressions.config.knob.maxShow) < RosUI.expressions.config.knob.maxShow)
            ? value / (RosUI.expressions.config.knob.max * 1.2) : 1.0;

        RosUI.api.setExpression(RosUI.expressions.current_face, intensity);
    },
    buildCrosshair: function (element, options) {
        var yaw = RosUI.ros.getMotorConfig("neck_base");
        var pitch = RosUI.ros.getMotorConfig("neck_pitch");

        $(element).crosshairsl($.extend({}, {
            xmin: Math.floor(RosUI.utilities.radToDeg(-1)),
            xmax: Math.ceil(RosUI.utilities.radToDeg(1)),
            xval: Math.round(RosUI.utilities.radToDeg(0)),
            ymin: Math.floor(RosUI.utilities.radToDeg(-0.6)),
            ymax: Math.ceil(RosUI.utilities.radToDeg(0.6)),
            yval: Math.round(RosUI.utilities.radToDeg(0)),
            change: function (e, ui) {
                RosUI.api.pointHead({
                    yaw: RosUI.utilities.degToRad(ui.xval),
                    pitch: RosUI.utilities.degToRad(-ui.yval)
                });
            } }, options));

        return element;
    }
};
