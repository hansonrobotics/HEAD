define(['application', './views/layout', './views/gestures', './views/cycles','./views/performances',
        './views/emotions', 'lib/api', 'entities/gesture','entities/performance', 'entities/cycle', 'entities/emotion'],
    function (App, LayoutView, GesturesView, CyclesView, PerformancesView, EmotionsView,  api) {
        var gestures = {
            index: function () {
                this.layoutView = new LayoutView();

                App.LayoutInstance.setTitle('Gestures');
                App.LayoutInstance.getRegion('content').show(this.layoutView);
                App.LayoutInstance.showNav();

                gestures.createGestureButtons();
                gestures.createCycleButtons();
                gestures.createEmotionSliders();
                gestures.createPerformanceButtons();
            },
            createGestureButtons: function () {
                var gestureCollection = new App.Entities.GestureCollection(),
                    gesturesView = new GesturesView({
                        collection: gestureCollection
                    });

                this.layoutView.getRegion('gestures').show(gesturesView);
                gestureCollection.fetch();
            },
            createCycleButtons: function () {
                var cycleCollection = new App.Entities.CycleCollection(),
                    cyclesView = new CyclesView({
                        collection: cycleCollection
                    });

                this.layoutView.getRegion('cycles').show(cyclesView);
                cycleCollection.fetch();
            },
            createPerformanceButtons: function () {
                var performanceCollection = new App.Entities.PerformanceCollection(),
                    performancesView = new PerformancesView({
                        collection: performanceCollection
                    });

                this.layoutView.getRegion('performances').show(performancesView);
                performanceCollection.fetch();
            },
            createEmotionSliders: function () {
                var emotionCollection = new App.Entities.EmotionCollection(),
                    emotionsView = new EmotionsView({
                        collection: emotionCollection
                    });

                this.layoutView.getRegion('emotions').show(emotionsView);
                emotionCollection.fetch();
            }
        };

        return gestures;
    });
