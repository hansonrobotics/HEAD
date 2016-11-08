define(['application', './views/dashboard', 'underscore', 'modules/performances/entities/performance_collection', 'entities/emotion_collection'],
    function (App, DashboardView, _, PerformanceCollection, EmotionCollection) {
        return {
            demo: function () {
                App.LayoutInstance.setTitle('Dashboard');

                var performances = new PerformanceCollection(),
                    emotions = new EmotionCollection(),
                    dashboardView = new DashboardView({
                        performances: performances,
                        emotions: emotions
                    });

                performances.fetch({
                    success: function () {
                        //expressionsCollection.each(function (model) {
                        //    var emotions = ['happy', 'sad', 'afraid', 'angry', 'surprised', 'Curious'];
                        //    if (_.indexOf(emotions, model.get('name')) == -1)
                        //        expressionsCollection.remove(model);
                        //});
                    }
                });
                emotions.fetch({
                    success: function () {
                        // var filtered = emotions.filter(function (expression) {
                        //     var expressions = [];
                        //     return _.indexOf(expressions, expression.get('name')) != -1;
                        // });
                        //
                        // emotions.reset(filtered);
                    }
                });

                App.LayoutInstance.getRegion('content').show(dashboardView);
            }
        };
    });
