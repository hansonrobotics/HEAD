define(['application', './views/layout', './views/performances', './views/timelines', './entities/performance',
        './entities/timeline', './entities/node'],
    function (App) {
        return {
            performances: function () {
                this.layoutView = new App.Performances.Views.Layout();

                App.LayoutInstance.setTitle('Interactions and Performances');
                App.LayoutInstance.getRegion('content').show(this.layoutView);

                this.performanceCollection = new App.Performances.Entities.PerformanceCollection();
                this.performanceCollection.testFetch();

                this.layoutView.getRegion('performances').show(new App.Performances.Views.Performances({
                    collection: this.performanceCollection
                }));

                var self = this;
                App.Performances.Views.on('performance:click', function (performance) {
                    self.timelineCollection = new App.Performances.Entities.TimelineCollection();
                    self.timelineCollection.testFetch();

                    self.layoutView.getRegion('timeline').show(new App.Performances.Views.Timelines({
                        collection: self.timelineCollection,
                        model: performance
                    }));
                });
            }
        };
    });
