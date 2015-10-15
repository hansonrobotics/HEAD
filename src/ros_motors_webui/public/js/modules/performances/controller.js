define(['application', 'backbone', 'lib/api', './views/layout', './views/performances', './entities/performance',
        './entities/node', './views/queue', './views/timelines'],
    function (App, Backbone, api) {
        return {
            performances: function () {
                var performanceCollection = new App.Performances.Entities.PerformanceCollection(),
                    layoutView = new App.Performances.Views.Layout(),
                    performanceQueueView = new App.Performances.Views.Queue({
                        layoutView: layoutView
                    });

                performanceCollection.fetch();
                api.blenderMode.enable();
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
                App.Performances.Views.off('performance:delete');
                App.Performances.Views.off('performance:add');
                App.Performances.Views.off('performances:save');

                // add events
                App.Performances.Views.on('performance:click', function (performance) {
                    // add to queue
                    performanceQueueView.addPerformance(performance);
                }).on('performances:save', function () {
                    performanceCollection.save();
                });

                App.Performances.Views.on('performance:add', function (performance) {
                    var timelinesView = new App.Performances.Views.Timelines({
                        collection: new Backbone.Collection(),
                        model: performance
                    });

                    // show configuration UI
                    layoutView.getRegion('timeline').destroy();
                    layoutView.getRegion('timeline').show(timelinesView);

                    timelinesView.enableEdit();
                });

                App.Performances.Views.on('performance:delete', function (performance) {
                    performanceQueueView.removePerformance(performance);
                });
            }
        };
    });
