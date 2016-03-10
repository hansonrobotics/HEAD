define(['marionette', 'tpl!./templates/animation_mode.tpl', 'lib/api', 'jquery'],
    function (Marionette, template, api, $) {
        return Marionette.ItemView.extend({
            ui: {
                modeButtons: '.app-gesture-pp',
                btOnButton: ".app-gesture-bt-on",
                btOffButton: ".app-gesture-bt-off",
                btFTButton: ".app-gesture-bt-ft",
                lsOnButton: ".app-gesture-ls-on",
                lsOffButton: ".app-gesture-ls-off"
            },
            template: template,
            events: {
                'click @ui.btOnButton': "btOn",
                'click @ui.btOffButton': "btOff",
                'click @ui.btFTButton': "btFT",
                'click @ui.modeButtons': "changePpMode",
                'click @ui.lsOnButton': "lsOn",
                'click @ui.lsOffButton': "lsOff"
            },
            btOn: function () {
                api.enableInteractionMode();
                api.setBTMode(api.btMo.C_ALL);
            },
            btOff: function () {
                api.disableInteractionMode();
            },
            lsOn: function () {
                api.setDynParam('/' + api.config.robot + '/anno_lipsync', 'lipsync', true)
            },
            lsOff: function () {
                api.setDynParam('/' + api.config.robot + '/anno_lipsync', 'lipsync', false)
            },
            btFT: function () {
                api.setBTMode(api.btModes.C_FACE | api.btModes.C_EYES);
                api.enableInteractionMode();
            },
            changePpMode: function (e) {
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
            }
        });
    });
