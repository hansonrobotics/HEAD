define(["application", "tpl!./templates/layout.tpl"], function (App, template) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Layout = Marionette.LayoutView.extend({
            template: template,
            ui: {
                startButton: '.app-gesture-demo-start',
                stopButton: '.app-gesture-demo-stop',
                gestures: '.app-gesture-buttons',
                emotions: '.app-emotion-sliders'
            },
            regions: {
                gestures: '.app-gesture-buttons',
                emotions: '.app-emotion-sliders'
            },
            events: {
                'click @ui.startButton': 'startDemo',
                'click @ui.stopButton': 'stopDemo'
            },
            onDestroy: function () {
                this.stopDemo();
            },
            setRandomGesture: function () {
                var emotionSliders = $('.app-slider', this.ui.emotions),
                    randomSlider = $(emotionSliders).get(Math.floor(Math.random() * $(emotionSliders).length));

                $(randomSlider).slider('value', Math.floor(Math.random() * 30) + 40);

                var gestureButtons = $('button', this.ui.gestures),
                    randomButton = $(gestureButtons).get(Math.floor(Math.random() * $(gestureButtons).length));

                $(randomButton).click();
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
