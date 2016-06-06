define(['application', 'tpl!./templates/system.tpl',  'bootbox', './check'], function (App, template, bootbox) {
    App.module('Start.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.System = Marionette.CompositeView.extend({
            initialize: function (options) {
                this.listenTo(this.collection, 'reset', this.refresh);
            },
            childView: App.Start.Views.Check,
            childViewContainer: '.app-checks',
            _base_config: {
                'system': {
                    'cpu': 0,
                    'mem': 0,
                    'total_mem': 16,
                },
                'robot': {
                    'current_name': 0,
                    'robots': ['sophia']
                },
                'status': {
                    'ros': 2,
                    'blender': 2,
                    'internet': 2,
                    'pololu': 2,
                    'usb2dynamixel': 2,
                    'camera': 2
                },
                'ros_status': {}
            },
            _statusClass: {
                0: {
                    label: 'OK',
                    class: 'label-success'
                },
                1: {
                    label: 'Err',
                    class: 'label-danger'
                },
                2: {
                    label: 'N/A',
                    class: 'label-warning'
                }
            },
            ui: {
                cpu: '.app-cpu',
                mem: '.app-mem',
                fps: '.app-fps',
                start: '.app-start-button',
                stop: '.app-stop-button'
            },
            events: {
                'click @ui.start': 'startRobot',
                'click @ui.stop': 'stopRobot'
            },
            _progress_bar_class: function (bar, val) {
                var cls = 'danger';
                switch (bar) {
                    case 'cpu':
                        if (val < 70) cls = 'warning';
                        if (val < 40) cls = 'success';
                        break;
                    case 'mem':
                        if (val < 80) cls = 'warning';
                        if (val < 60) cls = 'success';
                        break;
                }
                return 'progress-bar-' + cls;
            },
            redirect: false,
            redirect_enabled: true,
            updateView: function () {
                var self = this;
                // Main checks
                this.$('.status-item').each(function (i, e) {
                    var id = $(e).data("status");

                    $(e).removeClass('label-success label-warning label-danger').
                        addClass(self._statusClass[self.config.status[id]].class).
                        text(self._statusClass[self.config.status[id]].label);
                });
                // Progress bars
                var sys = this.config.system;
                var cls = 'progress-bar-success progress-bar-warning progress-bar-danger';
                this.ui.cpu.text(sys.cpu + "%");
                $(this.ui.cpu).width(sys.cpu + "%").removeClass(cls).addClass(this._progress_bar_class('cpu', sys.mem));
                this.ui.mem.text(sys.mem + "%");
                $(this.ui.mem).width(sys.mem + "%").removeClass(cls).addClass(this._progress_bar_class('mem', sys.mem));
                // Control buttons
                this.$('.app-software-control').hide();
                console.log(this.config.software)
                switch (this.config.software){
                    case 0: $('.app-software-stopped').show(); break;
                    case 1:
                        $('.app-software-starting').show();
                        // Set Redirect timer> Should have oiption to cancel redirect.
                        if (this.redirect_enabled && !this.redirect){
                            this.redirect = setTimeout(function(){
                                self.redirect = false;
                                window.location.port = 8000;
                            }, 3000);
                        }
                        break;
                    case 2: $('.app-software-started').show(); break;
                }

            },
            refresh: function () {
                var self = this;
                clearInterval(this.statusInterval);
                this.config = _.extend({}, this._base_config,  this.collection.config);
                this.updateView();
                this.statusInterval = setInterval(function(){self.collection.fetch()}, 5000);
            },
            startRobot: function () {
                var start = true;
                var el = false;
                var self = this;
                $(this.$el).find('.app-status.label-danger').each(function(i,e){
                    var str = $(e).parent().clone().children().remove().end().text().trim();
                    if (el) el += ", " + str;
                    else el = str;
                    start = false;
                });
                if (!start){
                    bootbox.confirm("Error with: "+el+". <br> Robot may no function properly. Do you want to start software anyway?", function (result) {
                        if (result){
                            self.collection.fetch("start");
                        }
                    });
                }else{
                    self.collection.fetch("start");
                }
            },
            stopRobot: function () {
                if (this.redirect){
                    clearTimeout(this.redirect);
                    this.redirtect = false;
                }
                this.collection.fetch("stop");

            },
            onRender: function () {
                this.config = _.extend({}, this._base_config,  this.collection.config);
                this.updateView();
            },
            onDestroy: function () {
                clearInterval(this.statusInterval);
            },
            template: template
        });
    });
    return App.module('Start.Views').System;
});
