define(['application', 'tpl!./templates/system.tpl', './ros_node'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.System = Marionette.CompositeView.extend({
            initialize: function (options) {
                console.log('init done');
                this.listenTo(this.collection, 'reset', this.render);
            },
            childView: App.Monitor.Views.RosNode,
            childViewContainer: '.app-ros-nodes',
            config :{
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
                'ros_status':{}
            },
            _statusClass:{
                0: {
                    label: 'OK',
                    class: 'label-success',
                },
                1: {
                    label: 'Err',
                    class: 'label-danger',
                },
                2: {
                    label: 'N/A',
                    class: 'label-warning',
                }
            },

            onRender: function () {
                this.config = _.extend(this.config, this.collection.config);
                var self = this;
                this.$('.status-item').each(function(i,e){
                   var id = $(e).data("status");
                   $(e).addClass(self._statusClass[self.config.status[id]].class).text(self._statusClass[self.config.status[id]].label);
                });
            },
            template: template
        });
    });
    return App.module('Monitor.Views').System;

});
