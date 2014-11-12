
function radToDeg(val) {return val*180/Math.PI;}
function degToRad(val) {return val*Math.PI/180;}

var KNOB_OPTS = {
    max: 55,
    maxShow: 11,
    fgColor: "#df691a",
    fgColorExtreme: "#df691a"
};

var btnObjs = []; //Holds an array of {name, label, element} objects representing expression buttons.
var curface = ''; //Holds the name of the last pressed button.

function getBtn(name) {
    for (var i = 0; i < btnObjs.length; i++) {
        if (btnObjs[i].name == name) {
            return btnObjs[i].element;
        }
    };
}

//Updates UI and inner model.
function setFace(name, label) {
    curface = name;
    $(".knobName").text(label);
}

//Updates UI and sends a ROS message.
function setKnobPos(value) {
    if (!value) {
        value = parseInt($(".dial").val())/KNOB_OPTS.maxShow*KNOB_OPTS.max;
    }
    $(".dial").val(value).trigger("change");
    var intensity = (Math.round(value/KNOB_OPTS.max*KNOB_OPTS.maxShow) < KNOB_OPTS.maxShow)
        ? value/(KNOB_OPTS.max*1.2) : 1.0
    RoboInterface.makeFaceExpr(curface, intensity);
}

//Create UI that requires a ROS connection.
function createUIros() {
    //Build preset buttons
    btnStack = $("#btnStack");
    btnStack.on("exprbtnclick", function(e, nameobj) {
        setFace(nameobj.name, nameobj.label);
        setKnobPos(KNOB_OPTS.max*1);
    }).on("success", function(e, _btnObjs) { //Note that btnObjs is a global variable
        btnObjs = _btnObjs.btnObjs; //Remove the object wrapper
        setFace(btnObjs[0].name, btnObjs[0].label);
        setKnobPos(KNOB_OPTS.max*1);
    });
    CommonUI.buildExpressionButtons(btnStack);
}

//Create UI that requires a loaded config file.
function createUIconf() {
    //Create crosshair
    CommonUI.buildCrosshairSlider(
        $(".crosshairsl"), {
            bgColor: "#485563",
            fgColor: "#fff"
        }
    ).on("change", function(e, ui){
            $(".crosshairsl")
                .crosshairsl("option", "xval", ui.xval)
                .crosshairsl("option", "yval", ui.yval);
        });

    //Create dial
    $(".dial").knob({
        min: 0, max: KNOB_OPTS.max,
        angleOffset: -125, angleArc: 275,
        width: 200, height: 200,
        thickness: 0.4,
        fgColor:KNOB_OPTS.fgColor, bgColor:"#444",
        draw: function() { //Extended the draw function to add a ring around the dial.
            if (parseInt($(".dial").val()) >=  Math.round(KNOB_OPTS.maxShow)) {
                paintSelection(this.o, KNOB_OPTS.fgColorExtreme);
            } else {
                paintSelection(this.o, KNOB_OPTS.fgColor);
            }

            this.g.lineWidth = 3;
            this.cursorExt = 0.2;

            //Override radius to fit the outer circle inside canvas
            this.radius = this.xy - this.lineWidth * 2 / 3 - this.g.lineWidth ;

            this.g.beginPath();
            this.g.strokeStyle = this.o.fgColor;
            this.g.arc(this.xy, this.xy, this.radius + 1 + this.lineWidth * 2 / 3, 0, 2 * Math.PI, false);
            this.g.stroke();
            return true;
        },
        change: setKnobPos,
        format: function(value) {
            return Math.round(value/KNOB_OPTS.max*KNOB_OPTS.maxShow);
        }
    });
    //Keep the input fields inside dials from being edited directly.
    $("input.dial").css("pointer-events", "none");
}

function paintSelection(knobObj, color) {
    knobObj.fgColor = color;
    $(".dial").css("color", color)
//    $(".knobName").css("color", color);
    $("#btnStack .btn").removeClass("highlight");
    var curBtn = getBtn(curface);
    if (curBtn) {
        curBtn.addClass("highlight");
    }
}

$(function() {
    $(".dial").val(0);
    RoboInterface.$.on("configload", createUIconf);
    RoboInterface.connect("ws://192.168.0.32:9090")

        .$.on("connection", function(e) {
            $("#status").css("color", "#88FF88");
            $("#status").html("&#9679; Connected");
            createUIros()
        }).on("error", function(e) {
            $("#status").css("color", "#FFAAAA");
            $("#status").html("&#9679; No connection");
        });
});