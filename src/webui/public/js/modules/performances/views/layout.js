define(['marionette', 'backbone', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in', 'lib/api', './performances',
        '../entities/performance_collection', './queue', './timelines', 'jquery', 'select2'],
    function (Marionette, Backbone, template, FadeInRegion, api, PerformancesView, PerformanceCollection, QueueView,
              TimelinesView, $) {
        return Marionette.LayoutView.extend({
            template: template,
            cssClass: 'app-performances-page',
            regions: {
                performances: {
                    el: '.app-performances-region',
                    regionClass: FadeInRegion
                },
                timeline: {
                    el: '.app-timeline-region',
                    regionClass: FadeInRegion
                },
                queue: {
                    el: '.app-performance-queue-container',
                    regionClass: FadeInRegion
                }
            },
            ui: {
                languageButton: '.app-language-select button',
                container: '.app-performances-page'
            },
            events: {
                'click @ui.languageButton': 'changeLanguage'
            },
            onShow: function () {
                var self = this;

                this.setFluidContainer(!!this.options.fluid);
                var performanceCollection = new PerformanceCollection(),
                    queueView = new QueueView({
                        layoutView: this,
                        performances: performanceCollection
                    });
                performanceCollection.fetch();
                // show performance buttons
                var performancesView = new PerformancesView({
                    collection: performanceCollection,
                    queueView: queueView
                });

                this.getRegion('performances').show(performancesView);

                // show queue
                this.getRegion('queue').show(queueView);

                performancesView.on('new', function (performance) {
                    var timelinesView = new TimelinesView({
                        collection: new Backbone.Collection(),
                        model: performance,
                        performances: performanceCollection,
                        readonly: false
                    });
                    self.getRegion('timeline').show(timelinesView);
                });
            },
            changeLanguage: function (e) {
                var language = $(e.target).data('lang');

                this.ui.languageButton.removeClass('active');
                $(e.target).addClass('active');

                api.setRobotLang(language);
            },
            setFluidContainer: function (enable) {
                if (enable)
                    this.ui.container.removeClass('container').addClass('container-fluid');
                else
                    this.ui.container.removeClass('container-fluid').addClass('container');
            }
        });
    });
