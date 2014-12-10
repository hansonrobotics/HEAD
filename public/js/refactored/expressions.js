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
        this.createUIros();
    },

    loadPage: function () {
        var blenderMessage, blinkMessage, treeMessage;

        blenderMessage = new ROSLIB.Message({data: 'Dummy'});
        RosUI.api.topics.cmdBlender.publish(blenderMessage);

        blinkMessage = new ROSLIB.Message({data: 'dmitry:stop'});
        RosUI.api.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_off'});
        RosUI.api.topics.cmdTree.publish(treeMessage);

        $('.expression-button.active').removeClass('active');

        RosUI.api.set_expression("happy", 0);
        RosUI.api.point_head();
    },

    initExpressions: function () {
        //Create crosshair
        CommonUI.buildCrosshairSlider(
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
                if (parseInt($(".dial").val()) >= Math.round(RosUI.expressions.config.knob.maxShow)) {
                    RosUI.expressions.paintSelection(this.o, RosUI.expressions.config.knob.fgColorExtreme);
                } else {
                    RosUI.expressions.paintSelection(this.o, RosUI.expressions.config.knob.fgColor);
                }

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
            change: RosUI.expressions.setKnobPos,
            format: function (value) {
                return Math.round(value / RosUI.expressions.config.knob.max * RosUI.expressions.config.knob.maxShow);
            }
        });
        //Keep the input fields inside dials from being edited directly.
        $("input.dial").css("pointer-events", "none");
    },
    createUIros: function () {
        //Build preset buttons
        var btnStack = $(".expression-buttons");
        btnStack.on("exprbtnclick", function (e, nameobj) {
            console.log('expression button clicked');

            RosUI.expressions.setFace(nameobj.name, nameobj.label);
            RosUI.expressions.setKnobPos(RosUI.expressions.config.knob.max);
        });

        CommonUI.buildExpressionButtons(btnStack);
    },
    setFace: function (name, label) {
        RosUI.expressions.current_face = name;
        $(".knobName").text(label);
        console.log('set face to: ' + RosUI.expressions.current_face);
    },
    setKnobPos: function (value) {
        if (!value)
            value = parseInt($(".dial").val()) / RosUI.expressions.config.knob.maxShow * RosUI.expressions.config.knob.max;

        $(".dial").val(value).trigger("change");

        var intensity = (Math.round(value / RosUI.expressions.config.knob.max *
            RosUI.expressions.config.knob.maxShow) < RosUI.expressions.config.knob.maxShow)
            ? value / (RosUI.expressions.config.knob.max * 1.2) : 1.0

        RosUI.api.set_expression(RosUI.expressions.current_face, intensity);
    },

    paintSelection: function (knobObj, color) {
        knobObj.fgColor = color;
        $(".dial").css("color", color);
        $(".knobName").css("color", color);
        $(".expression-button").removeClass("active");

        if (RosUI.expressions.current_face)
            $(".expression-button." + RosUI.expressions.current_face).addClass("active");
    }
};
