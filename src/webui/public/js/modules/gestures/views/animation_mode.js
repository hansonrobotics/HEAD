define(['marionette', 'tpl!./templates/animation_mode.tpl', 'lib/api', 'jquery'],
    function (Marionette, template, api, $) {
        return Marionette.ItemView.extend({
            ui: {
                modeButtons: '.app-gesture-pp',
                btOnButton: ".app-gesture-bt-on",
                btOffButton: ".app-gesture-bt-off"
            },
            template: template,
            events: {
                'click @ui.btOnButton': "btOn",
                'click @ui.btOffButton': "btOff",
                'click @ui.modeButtons': "changePpMode"
            },
            btOn: function () {
                api.enableInteractionMode();
            },
            btOff: function () {
                api.disableInteractionMode();
            },
            changePpMode: function (e) {
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
            }
        });
    });
