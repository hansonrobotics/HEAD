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
            initialize: function (options) {
                this.mergeOptions(options, ['readonly', 'autoplay']);
            },
            onShow: function () {
                this.setFluidContainer(!!this.options.fluid);
                this.performanceCollection = new PerformanceCollection();

                var self = this,
                    queueView = new QueueView({
                        performances: self.performanceCollection,
                        readonly: this.readonly,
                        autoplay: this.autoplay
                    }),
                    performancesView = new PerformancesView({
                        collection: self.performanceCollection,
                        queueView: queueView,
                        readonly: this.readonly,
                        autoplay: this.autoplay
                    });

                self.getRegion('queue').show(queueView);
                self.getRegion('performances').show(performancesView);

                performancesView.on('new', queueView.editPerformance, queueView);
                this.performanceCollection.fetch({
                    success: function () {
                        queueView.showCurrent();
                    }
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
