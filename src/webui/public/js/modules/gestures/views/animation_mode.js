define(['marionette', 'tpl!./templates/animation_mode.tpl', 'lib/api', 'jquery'],
    function (Marionette, template, api, $) {
        return Marionette.ItemView.extend({
            ui: {
                modeButtons: '.app-gesture-pp'
            },
            template: template,
            events: {
                'click @ui.modeButtons': "changePpMode"
            },
            changePpMode: function (e) {
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
            }
        });
    });