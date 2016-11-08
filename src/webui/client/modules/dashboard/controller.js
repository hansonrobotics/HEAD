define(['application', './views/dashboard', 'underscore', 'modules/performances/entities/performance_collection', 'entities/expression'],
    function (App, DashboardView, _, PerformanceCollection) {
        return {
            demo: function () {
                App.LayoutInstance.setTitle('Dashboard');

                var performanceCollection = new PerformanceCollection(),
                    expressionsCollection = new App.Entities.ExpressionCollection(),
                    dashboardView = new DashboardView({
                        performances: performanceCollection,
                        expressions: expressionsCollection
                    });

                performanceCollection.fetch({
                    success: function () {
                        //expressionsCollection.each(function (model) {
                        //    var emotions = ['happy', 'sad', 'afraid', 'angry', 'surprised', 'Curious'];
                        //    if (_.indexOf(emotions, model.get('name')) == -1)
                        //        expressionsCollection.remove(model);
                        //});
                    }
                });
                expressionsCollection.fetch({
                    success: function () {
                        var filtered = expressionsCollection.filter(function (expression) {
                            var expressions = ['Neutral', 'Smile', './sto   ', 'Afraid', 'Kiss', 'Disgusted'];
                            return _.indexOf(expressions, expression.get('name')) != -1;
                        });

                        expressionsCollection.reset(filtered);
                    }
                });

                App.LayoutInstance.getRegion('content').show(dashboardView);
            }
        };
    });
