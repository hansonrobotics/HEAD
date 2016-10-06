define(['marionette', 'backbone', './templates/layout.tpl', 'lib/regions/fade_in', 'lib/api', './performances',
        '../entities/performance_collection', '../entities/performance', './queue', './timelines', 'jquery',
        'underscore', 'select2', 'select2-css'],
    function (Marionette, Backbone, template, FadeInRegion, api, PerformancesView, PerformanceCollection, Performance,
              QueueView, TimelinesView, $, _) {
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
                this.setFluidContainer(!!this.options.fluid);
                this.performanceCollection = new PerformanceCollection();

                var self = this,
                    current = new Performance(),
                    showViews = function (ids) {
                        var queueView = new QueueView({
                                layoutView: self,
                                performances: self.performanceCollection,
                                sequence: ids
                            }),
                            performancesView = new PerformancesView({
                                collection: self.performanceCollection,
                                queueView: queueView
                            });

                        self.showTimeline(current, {readonly: true});
                        self.getRegion('queue').show(queueView);
                        self.getRegion('performances').show(performancesView);

                        performancesView.on('new', self.showTimeline, self);
                    };

                this.performanceCollection.fetch({
                    success: function () {
                        current.fetchCurrent({
                            success: function (response) {
                                showViews(_.pluck(response.performances, 'id'));

                            },
                            error: function () {
                                showViews([]);
                            }
                        });
                    },
                    error: function () {
                        showViews([]);
                    }
                });
            },
            showTimeline: function (performance, options) {
                var timelinesView = new TimelinesView(_.extend({
                    collection: new Backbone.Collection(),
                    model: performance,
                    performances: this.performanceCollection,
                    readonly: false
                }, options));
                this.getRegion('timeline').show(timelinesView);
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
