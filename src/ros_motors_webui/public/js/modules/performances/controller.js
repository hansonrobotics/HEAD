define(['application', 'backbone', './views/layout', './views/performances', './entities/performance',
        './entities/node', './views/queue'],
    function (App, Backbone) {
        return {
            performances: function () {
                var performanceCollection = new App.Performances.Entities.PerformanceCollection(),
                    layoutView = new App.Performances.Views.Layout(),
                    performanceQueueView = new App.Performances.Views.Queue({
                        layoutView: layoutView
                    });

                performanceCollection.fetch();

                // show page
                App.LayoutInstance.setTitle('Interactions and Performances');
                App.LayoutInstance.getRegion('content').show(layoutView);

                // show performance buttons
                layoutView.getRegion('performances').show(new App.Performances.Views.Performances({
                    collection: performanceCollection
                }));

                // show queue
                layoutView.getRegion('queue').show(performanceQueueView);

                // remove previous event handlers
                App.Performances.Views.off('performance:click');
                App.Performances.Views.off('performances:save');

                // add events
                App.Performances.Views.on('performance:click', function (performance) {
                    // add to queue
                    performanceQueueView.addPerformance(performance);
                }).on('performances:save', function () {
                    performanceCollection.save();
                });
            }
        };
    });
