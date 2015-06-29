define(["application", "tpl!./templates/layout.tpl"], function (App, template) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Layout = Marionette.LayoutView.extend({
            template: template,
            ui: {
                startButton: '.app-gesture-demo-start',
                stopButton: '.app-gesture-demo-stop',
                gestures: '.app-gesture-buttons',
                emotions: '.app-emotions-container'
            },
            regions: {
                gestures: '.app-gesture-buttons',
                emotions: '.app-emotions-container'
            },
            events: {
                'click @ui.startButton': 'startDemo',
                'click @ui.stopButton': 'stopDemo'
            },
            onDestroy: function () {
                this.stopDemo();
            },
            setRandomGesture: function () {
                $('.app-duration-slider', this.ui.emotions).slider('value', Math.floor(Math.random() * 100));
                $('.app-magnitude-slider', this.ui.emotions).slider('value', Math.floor(Math.random() * 100));

                // click random emotion
                var emotionButtons = $('button', this.ui.emotions),
                    randomEmotionButton = $(emotionButtons).get(Math.floor(Math.random() * $(emotionButtons).length));
                $(randomEmotionButton).click();

                // click random gesture
                var gestureButtons = $('button', this.ui.gestures),
                    randomGestureButton = $(gestureButtons).get(Math.floor(Math.random() * $(gestureButtons).length));
                $(randomGestureButton).click();
            },
            startDemo: function () {
                this.ui.startButton.addClass('active');
                this.ui.stopButton.removeClass('active');

                var self = this;
                this.demo = setInterval(function () {
                    self.setRandomGesture();
                }, 2000);
            },
            stopDemo: function () {
                this.ui.startButton.removeClass('active');
                this.ui.stopButton.addClass('active');

                if (this.demo)
                    clearTimeout(this.demo);
            }
        });
    });

    return App.module('Gestures.Views').Layout;
});
