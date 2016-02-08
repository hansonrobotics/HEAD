define(["application", "tpl!./templates/layout.tpl", "lib/api"], function (App, template, api) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Layout = Marionette.LayoutView.extend({
            template: template,
            ui: {
                btOnButton: ".app-gesture-bt-on",
                btOffButton: ".app-gesture-bt-off",
                ppOffButton: ".app-gesture-bt-off",
                puppeteeringButtons: '.app-gesture-pp'
            },
            regions: {
                performances: '.app-performance-buttons',
                cycles: '.app-cycle-buttons',
                startButton: '.app-gesture-demo-start',
                stopButton: '.app-gesture-demo-stop',
                gestures: '.app-gesture-buttons',
                emotions: '.app-emotions-container'
            },
            events: {
                'click @ui.btOnButton': "btOn",
                'click @ui.btOnStageButton': "btOnStage",
                'click @ui.btEmotionsOffButton': "btEmotionsOff",
                'click @ui.btGesturesOffButton': "btGesturesOff",
                'click @ui.btOffButton': "btOff",
                'click @ui.sayIntroButton': "sayIntro",
                'click @ui.puppeteeringButtons': "changePpMode"
            },
            btOn: function() {
                api.enableInteractionMode();
            },
            btOff: function() {
                api.disableInteractionMode();
            },
            sayIntro: function() {
                api.sendChatMessage('start demo');
            },
            changePpMode: function (e) {
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
            }
        });
    });

    return App.module('Gestures.Views').Layout;
});
