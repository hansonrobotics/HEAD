define(['application', './views/layout', './views/performances', './views/timelines', './entities/performance',
        './entities/timeline'],
    function (App) {
        return {
            performances: function () {
                this.layoutView = new App.Performances.Views.Layout();

                App.LayoutInstance.setTitle('Interactions and Performances');
                App.LayoutInstance.getRegion('content').show(this.layoutView);

                this.timelineCollection = new App.Performances.Entities.TimelineCollection();
                this.performanceCollection = new App.Performances.Entities.PerformanceCollection();
                this.performanceCollection.testFetch();
                this.timelineCollection.testFetch();

                this.layoutView.getRegion('performances').show(new App.Performances.Views.Performances({
                    collection: this.performanceCollection
                }));
                this.layoutView.getRegion('timeline').show(new App.Performances.Views.Timelines({
                    collection: this.timelineCollection
                }));
            }
        };
    });
