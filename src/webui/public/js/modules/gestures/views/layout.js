define(["marionette", "tpl!./templates/layout.tpl", "lib/api", './gestures', './cycles', './performances',
        './emotions', './animation_mode'],
    function (Marionette, template, api, GesturesView, CyclesView, PerformancesView, EmotionsView,
              AnimationModeView) {
        return Marionette.LayoutView.extend({
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
                this.cyclesView = new CyclesView();
                this.getRegion('cycles').show(this.cyclesView);
            },
            createPerformanceButtons: function () {
                this.performancesView = new PerformancesView();
                this.getRegion('performances').show(this.performancesView);
            },
            createEmotionSliders: function () {
                this.emotionsView = new EmotionsView();
                this.getRegion('emotions').show(this.emotionsView);
            },
            createAnimationModeButtons: function () {
                this.animationModeView = new AnimationModeView();
                this.getRegion('animation_mode').show(this.animationModeView);
            }
        });
    });
