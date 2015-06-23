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
                gestures: '.app-gesture-buttons',
                emotions: '.app-emotion-sliders'
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
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_on'}));
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
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
            },
            sayIntro: function() {
                api.sendChatMessage('start demo');
            }
        });
    });

    return App.module('Gestures.Views').Layout;
});
