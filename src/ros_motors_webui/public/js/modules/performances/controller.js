define(['application', 'backbone', './views/layout', './views/performances', './views/timelines', './entities/performance',
        './entities/node'],
    function (App, Backbone) {
        return {
            performances: function () {
                var layoutView = new App.Performances.Views.Layout(),
                    performanceCollection = new App.Performances.Entities.PerformanceCollection();
                performanceCollection.fetch();

                App.LayoutInstance.setTitle('Interactions and Performances');
                App.LayoutInstance.getRegion('content').show(layoutView);

                layoutView.getRegion('performances').show(new App.Performances.Views.Performances({
                    collection: performanceCollection
                }));

                // remove previous event handlers
                App.Performances.Views.off('performance:click');
                App.Performances.Views.off('performances:save');

                // add events
                App.Performances.Views.on('performance:click', function (performance) {
                    layoutView.getRegion('timeline').destroy();
                    layoutView.getRegion('timeline').show(new App.Performances.Views.Timelines({
                        collection: new Backbone.Collection(),
                        model: performance
                    }));
                }).on('performances:save', function () {
                    performanceCollection.save();
                });
            }
        };
    });
