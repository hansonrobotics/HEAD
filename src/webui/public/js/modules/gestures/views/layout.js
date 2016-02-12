define(["application", "tpl!./templates/layout.tpl", "lib/api", './gestures', './cycles', './performances',
        './emotions', './animation_mode', 'entities/performance_collection', 'entities/cycle_collection', 'entities/emotion_collection'],
    function (App, template, api, GesturesView, CyclesView, PerformancesView, EmotionsView,
              AnimationModeView, PerformanceCollection, CycleCollection, EmotionCollection) {
        App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Layout = Marionette.LayoutView.extend({
                template: template,
                ui: {
                    btOnButton: ".app-gesture-bt-on",
                    btOffButton: ".app-gesture-bt-off"
                },
                regions: {
                    performances: '.app-performance-buttons',
                    cycles: '.app-cycle-buttons',
                    startButton: '.app-gesture-demo-start',
                    stopButton: '.app-gesture-demo-stop',
                    gestures: '.app-gesture-buttons',
                    emotions: '.app-emotions-container',
                    animation_mode: '.app-animation-mode-region'
                },
                events: {
                    'click @ui.btOnButton': "btOn",
                    'click @ui.btOnStageButton': "btOnStage",
                    'click @ui.btEmotionsOffButton': "btEmotionsOff",
                    'click @ui.btGesturesOffButton': "btGesturesOff",
                    'click @ui.btOffButton': "btOff",
                    'click @ui.sayIntroButton': "sayIntro"
                },
                onRender: function () {
                    api.blenderMode.enable();

                    this.createGestureButtons();
                    this.createCycleButtons();
                    this.createEmotionSliders();
                    this.createPerformanceButtons();
                    this.createAnimationModeButtons();
                },
                btOn: function () {
                    api.enableInteractionMode();
                },
                btOff: function () {
                    api.disableInteractionMode();
                },
                sayIntro: function () {
                    api.sendChatMessage('start demo');
                },
                createGestureButtons: function () {
                    this.gesturesView = new GesturesView();
                    this.getRegion('gestures').show(this.gesturesView);
                },
                createCycleButtons: function () {
                    this.cycleCollection = new CycleCollection();
                    this.cyclesView = new CyclesView({
                        collection: this.cycleCollection
                    });

                    this.getRegion('cycles').show(this.cyclesView);
                    this.cycleCollection.fetch();
                },
                createPerformanceButtons: function () {
                    this.performanceCollection = new PerformanceCollection();
                    this.performancesView = new PerformancesView({
                        collection: this.performanceCollection
                    });

                    this.getRegion('performances').show(this.performancesView);
                    this.performanceCollection.fetch();
                },
                createEmotionSliders: function () {
                    this.emotionCollection = new EmotionCollection();
                    this.emotionsView = new EmotionsView({
                        collection: this.emotionCollection
                    });

                    this.getRegion('emotions').show(this.emotionsView);
                    this.emotionCollection.fetch();
                },
                createAnimationModeButtons: function () {
                    this.animationModeView = new AnimationModeView();
                    this.getRegion('animation_mode').show(this.animationModeView);
                }
            });
        });

        return App.module('Gestures.Views').Layout;
    });
