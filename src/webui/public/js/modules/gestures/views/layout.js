define(["application", "tpl!./templates/layout.tpl", "lib/api"], function (App, template, api) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Layout = Marionette.LayoutView.extend({
            template: template,
            ui: {
                btOnButton: ".app-gesture-bt-on",
                btOnStageButton: ".app-gesture-bt-on-stage",
                btEmotionsOffButton: ".app-gesture-bt-emotions-off",
                btGesturesOffButton: ".app-gesture-bt-gestures-off",
                btOffButton: ".app-gesture-bt-off",
                sayIntroButton: ".app-gesture-say-intro"
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
                'click @ui.sayIntroButton': "sayIntro"
            },
            btOn: function() {
                api.enableInteractionMode();
            },
            btOnStage: function() {
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_on_stage'}));
            },
            btEmotionsOff: function() {
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'emotion_off'}));
            },
            btGesturesOff: function() {
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'gesture_off'}));
            },
            btOff: function() {
                api.disableInteractionMode();
            },
            sayIntro: function() {
                api.sendChatMessage('start demo');
            }
        });
    });

    return App.module('Gestures.Views').Layout;
});
