define(['application', './templates/system.tpl', './ros_node'], function (App, template, RosNode) {
    return Marionette.CompositeView.extend({
        initialize: function (options) {
            this.listenTo(this.collection, 'reset', this.refresh);
        },
        childView: RosNode,
        childViewContainer: '.app-ros-nodes',
        _base_config: {
            'system': {
                'cpu': 0,
                'mem': 0,
                'total_mem': 16,
                'fps': 0
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
            fps: '.app-fps'
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
                case 'fps':
                    if (val > 30) cls = 'warning';
                    if (val > 40) cls = 'success';
                    break;
            }
            return 'progress-bar-' + cls;
        },
        updateView: function () {
            var self = this;
            // Main checks
            this.$('.status-item').each(function (i, e) {
                var id = $(e).data("status");

                $(e).removeClass('label-success label-warning label-danger').addClass(self._statusClass[self.config.status[id]].class).text(self._statusClass[self.config.status[id]].label);
            });
            // Progress bars
            var sys = this.config.system;
            var cls = 'progress-bar-success progress-bar-warning progress-bar-danger';
            this.ui.cpu.text(sys.cpu + "%");
            $(this.ui.cpu).width(sys.cpu + "%").removeClass(cls).addClass(this._progress_bar_class('cpu', sys.mem));
            this.ui.mem.text(sys.mem + "%");
            $(this.ui.mem).width(sys.mem + "%").removeClass(cls).addClass(this._progress_bar_class('mem', sys.mem));
            this.ui.fps.text(sys.fps);
            $(this.ui.fps).width(sys.fps + "%").removeClass(cls).addClass(this._progress_bar_class('fps', sys.fps));
        },
        refresh: function () {
            var self = this;
            clearInterval(this.statusInterval);
            this.config = _.extend({}, this._base_config, this.collection.config);
            this.updateView();
            this.statusInterval = setInterval(function () {
                self.collection.fetch()
            }, 2000);
        },
        onRender: function () {
            this.config = _.extend({}, this._base_config, this.collection.config);
            this.updateView();
        },
        onDestroy: function () {
            clearInterval(this.statusInterval);
        },
        template: template
    });
});
