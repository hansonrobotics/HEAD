define(['marionette', 'tpl!./templates/controls.tpl', 'lib/api', 'roslib'],
    function (Marionette, template, api, ROSLIB) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                modeButtons: '.app-animation-mode-button'
            },
            events: {
                'click @ui.modeButtons': 'changePpMode'
            },
            changePpMode: function (e) {
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
            }
        });
    });
