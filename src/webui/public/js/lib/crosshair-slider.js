define(['jquery', 'jquery-ui'], function ($) {
    $.widget("custom.crosshairsl", {
        options: {
            fgColor: "#fff",
            bgColor: "#485563",
            xmin: -50, xmax: 50, xval: 0,
            ymin: -50, ymax: 50, yval: 0,
            change: null
        },
        _create: function () {
            if (this.element.width() == 0) this.element.width(100);
            if (this.element.height() == 0) this.element.height(100);

            this.element
                .css("background-color", this.options.bgColor)
                .css("border", "solid 1px");

            this.horline = $("<div/>")
                .css("width", "100%")
                .css("height", "1px")
                .css("border-top", "solid 1px")
                .addClass("crosshairsl-line")
                .appendTo(this.element);

            this.verline = $("<div/>")
                .css("width", "1px")
                .css("height", "100%")
                .css("border-left", "solid 1px")
                .addClass("crosshairsl-line")
                .appendTo(this.element);

            this.element.find(".crosshairsl-line")
                .css("position", "relative");

            this._on(this.element, {
                mousedown: "_startDrag",
                touchstart: "_startDrag"
            });

            this._on(window, {
                mouseup: "_stopDrag",
                touchend: "_stopDrag"
            });

            this._refresh();
        },
        _updateCrosshair: function () {
            var opts = this.options;
            this.verline.css("left", (opts.xval - opts.xmin) / (opts.xmax - opts.xmin) * this.element.width());
            this.horline.css("top", (opts.yval - opts.ymin) / (opts.ymax - opts.ymin) * this.element.height());
        },
        //Called when created, and later when changing options by the user
        _refresh: function () {
            this.element.find(".crosshairsl-line")
                .css("border-color", this.options.fgColor);

            this._updateCrosshair();
        },
        _startDrag: function (e) {
            this._on($("body"), {
                touchmove: "_dragMove",
                mousemove: "_dragMove"
            });
            this._dragMove(e);
        },
        _stopDrag: function (e) {
            this._off($("body"), "touchmove mousemove");
        },
        _dragMove: function (e) {
            e.preventDefault();

            if (!e.pageX || !e.pageY) { //Touch compatibility
                e = e.originalEvent.touches[0];
            }

            //Update options object
            var offset = this.element.offset();
            var opts = this.options;
            this._setXval((e.pageX - offset.left) / this.element.width() * (opts.xmax - opts.xmin) + opts.xmin);
            this._setYval((e.pageY - offset.top) / this.element.height() * (opts.ymax - opts.ymin) + opts.ymin);

            this._updateCrosshair();

            this._trigger("change", e, {xval: opts.xval, yval: opts.yval});
            this.element.trigger("change", {xval: opts.xval, yval: opts.yval});
        },
        _destroy: function () {
            this.horline.remove();
            this.verline.remove();

            this.element
                .css("background-color", "transparent")
                .css("border", "None");
        },
        //_setOptions is called with a hash of all options that are changing
        _setOptions: function () {
            this._superApply(arguments);
            this._refresh();
        },
        //_setOption is called for each individual option that is changing
        _setOption: function (key, value) {
            //Prevent out of bound values
            if (key == "xval") {
                this._setXval(value);
                return;
            } else if (key == "yval") {
                this._setYval(value);
                return;
            }
            this._super(key, value);
        },
        _setXval: function (val) {
            var opts = this.options;
            opts.xval = Math.min(Math.max(val, opts.xmin), opts.xmax);
        },
        _setYval: function (val) {
            var opts = this.options;
            opts.yval = Math.min(Math.max(val, opts.ymin), opts.ymax);
        }
    });
});
