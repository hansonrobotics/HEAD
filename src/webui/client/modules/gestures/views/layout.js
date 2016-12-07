define(["marionette", "./templates/layout.tpl", "lib/api", './animations', './cycles', './performances',
        './poses', './animation_mode', 'roslib'],
    function (Marionette, template, api, GesturesView, CyclesView, PerformancesView, EmotionsView,
              AnimationModeView, ROSLIB) {
        return Marionette.View.extend({
            template: template,
            regions: {
                performances: '.app-performance-buttons',
                cycles: '.app-cycle-buttons',
                startButton: '.app-gesture-demo-start',
                stopButton: '.app-gesture-demo-stop',
                gestures: '.app-gesture-buttons',
                emotions: '.app-emotions-container',
                animationMode: '.app-animation-mode-region'
            },
            onRender: function () {
                api.blenderMode.enable();

                this.createGestureButtons();
                this.createCycleButtons();
                this.createEmotionSliders();
                this.createPerformanceButtons();
                this.createAnimationModeButtons();
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
            changePpMode: function(e){
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
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
                this.getRegion('animationMode').show(this.animationModeView);
            }
        });
    });
