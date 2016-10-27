define(['marionette', './templates/animation_mode.tpl', 'lib/api', 'jquery', 'roslib'],
    function (Marionette, template, api, $, ROSLIB) {
        return Marionette.ItemView.extend({
            ui: {
                modeButtons: '.app-gesture-pp',
                btOnButton: ".app-gesture-bt-on",
                btOffButton: ".app-gesture-bt-off",
                btFTButton: ".app-gesture-bt-ft",
                lsOnButton: ".app-gesture-ls-on",
                lsOffButton: ".app-gesture-ls-off",
                btFTOffButton: ".app-gesture-bt-ft-off",
                behaviorContainer: '.app-behavior-container',
                lipsyncContainer: '.app-lipsync-container',
                modeContainer: '.app-mode-container'
            },
            template: template,
            events: {
                'click @ui.btOnButton': "btOn",
                'click @ui.btOffButton': "btOff",
                'click @ui.btFTButton': "btFT",
                'click @ui.btFTOffButton': "btFTOff",
                'click @ui.modeButtons': "changePpMode",
                'click @ui.lsOnButton': "lsOn",
                'click @ui.lsOffButton': "lsOff"
            },
            initialize: function (options) {
                this.mergeOptions(options, ['settings']);
                if (!this.settings || !this.settings.constructor === Array)
                    this.settings = ['behavior', 'mode', 'lipsync'];
            },
            onAttach: function () {
                var cssClass = 'col-md-' + 12 / this.settings.length;

                this.ui.behaviorContainer.addClass(cssClass);
                this.ui.modeContainer.addClass(cssClass);
                this.ui.lipsyncContainer.addClass(cssClass);

                if (!_.includes(this.settings, 'behavior'))
                    this.ui.behaviorContainer.hide();

                if (!_.includes(this.settings, 'mode'))
                    this.ui.modeContainer.hide();

                if (!_.includes(this.settings, 'lipsync'))
                    this.ui.lipsyncContainer.hide();
            },
            btOn: function () {
                api.enableInteractionMode();
                api.setBTMode(api.btModes.C_ALL);
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
            btFTOff: function () {
                api.setBTMode(api.btModes.C_ALL - (api.btModes.C_FACE | api.btModes.C_EYES));
                api.enableInteractionMode();
            },
            changePpMode: function (e) {
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
            }
        });
    });
