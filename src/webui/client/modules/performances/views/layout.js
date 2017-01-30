define(['marionette', 'backbone', './templates/layout.tpl', 'lib/regions/fade_in', 'lib/api', './performances',
        '../entities/performance_collection', '../entities/performance', './queue', './timelines', 'jquery',
        'underscore', '../entities/queue_item', 'select2', 'select2-css'],
    function (Marionette, Backbone, template, FadeInRegion, api, PerformancesView, PerformanceCollection, Performance,
              QueueView, TimelinesView, $, _, QueueItem) {
        return Marionette.View.extend({
            template: template,
            className: 'app-performances-page',
            regions: {
                performances: {
                    el: '.app-performances-region',
                    regionClass: FadeInRegion
                },
                queue: { 
                    el: '.app-queue-region',
                },
                timeline: {
                    el: '.app-timeline-region',
                    regionClass: FadeInRegion
                }
            },
            ui: {
                languageButton: '.app-language-select button',
                container: '.app-performances-region',
                queueContainer: '.app-queue-container',
                queue: '.app-performance-queue',
                performances: '.app-performance-queue .app-performance',
                performanceTemplate: '.app-performance-template',
                clearButton: '.app-clear',
                emptyNotice: '.app-empty-notice'
            },
            events: {
                'click @ui.languageButton': 'changeLanguage',
                'click @ui.clearButton': 'clearClick'
            },
            initialize: function (options) {
                this.mergeOptions(options, ['editing', 'autoplay', 'dir', 'nav', 'readonly', 'hideQueue', 'disableSaving']);
            },
            onRender: function () {
                // fluid by default
                this.setFluidContainer(this.fluid || typeof this.fluid == 'undefined');
                this.performances = new PerformanceCollection();
                this.queueCollection = new PerformanceCollection();
                this.listenTo(this.queueCollection, 'add remove reset', function() {
                    if (!this.editting)
                        this.updateTimeline();
                });

                let self = this,
                    queueView = new QueueView({ 
                        collection: this.queueCollection,
                        readonly: this.readonly,
                        layoutView: this
                    }),
                    performancesView = new PerformancesView({
                        collection: self.performances,
                        layoutView: this,
                        readonly: this.readonly,
                        autoplay: this.autoplay,
                        dir: this.dir,
                        nav: this.nav
                    });

                self.getRegion('queue').show(queueView); 
                self.getRegion('performances').show(performancesView);

                this.listenTo(performancesView, 'new', this.editPerformance);
                this.performances.fetch({
                    reset: true,
                    success: function (response, collection) {
                        self.showCurrent();
                    }
                });

                if (this.hiddenQueue) {
                    this.ui.queueContainer.hide();
                }

                if (this.options.sequence)
                    this.showSequence(this.options.sequence);

                if (this.readonly) {
                    this.runningPerformances = new Performance();
                    this.runningPerformances.enableSync(function (performances) {
                        self.showSequence(_.map(performances, 'id'), true);
                    });
                }
            },
            onDestroy: function () {
                if (this.readonly) this.runningPerformances.disableSync();
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
            },
            showSequence: function (sequence, skipTimelineUpdate) {
                let self = this;

                if (!_.isEqual(this._getPerformanceIds(), sequence)) {
                    this.clearQueue();
                    if (sequence instanceof Array && this.performances) {
                        _.each(sequence, function (id) {
                            let model = self.performances.get(id);
                            if (model) self.addPerformance(model, true);
                        });
                    }

                    if (!skipTimelineUpdate) this.updateTimeline();
                }
            },
            showCurrent: function () {
                let self = this,
                    current = new Performance();

                current.fetchCurrent({
                    success: function (response) {
                        self._showTimeline({
                            model: current,
                            running: response['running'],
                            paused: response['paused'],
                            current_time: response['current_time'],
                            readonly: true
                        });

                        let ids = _.map(response['performances'], 'id');
                        self.showSequence(ids, true);
                        self.fetchPerformances(ids);
                    }
                });
            },
            fetchPerformances: function (ids) {
                let self = this;

                // fetch full info for performances in the sequence
                _.each(ids, function (id) {
                    let p = self.performances.get(id);
                    if (p && p.nodes.isEmpty()) p.fetch();
                });
            },
            findItemByTime: function(time) {
                let offset = 0;
                return this.queueCollection.find(function(item) {
                    let match = time >= offset;
                    offset += item.get('performance').getDuration();
                    return match && time < offset;
                });
            },
            editCurrent: function() {
                let item = this.findItemByTime(this.time);
                if (item) this.editPerformance(item.get('performance'));
            },
            getItemIndex: function(performance) {
                return this.queueCollection.findIndex(function(item) {
                    return item.get('performance') === performance;
                });
            },
            editPrevious: function() {
                let self = this;
                if (this.timelinesView) {
                    let i = this.getItemIndex(this.timelinesView.model);
                    if (i > 0) this.editPerformance(this.queueCollection.at(i - 1).get('performance'));
                }
            },
            editNext: function() {
                let self = this;
                if (this.timelinesView) {
                    let i = this.getItemIndex(this.timelinesView.model);
                    if (i >= 0 && i < this.queueCollection.length - 1) this.editPerformance(this.queueCollection.at(i + 1).get('performance'));
                }
            },
            editPerformance: function (performance) {
                this._showTimeline({
                    model: performance,
                    readonly: false
                });
            },
            getPerformanceStartTime: function (performance) {
                let startTime = 0;

                this.queueCollection.find(function (item) {
                    let p = item.get('performance'),
                        found = p === performance;
                    if (!found) startTime += p.getDuration();
                    return found;
                });

                return startTime;
            },
            play: function(performance) {
                console.log(performance)
                if (!this.timelinesView || !this.timelinesView.readonly)
                    this.updateTimeline();
                this.timelinesView.run(this.getPerformanceStartTime(performance));
            },
            setTime: function(performance) {
                if (this.editting) {
                    this.editPerformance(performance);
                } else {
                    if (!this.timelinesView) this.updateTimeline();
                    this.timelinesView.moveIndicator(this.getPerformanceStartTime(performance));
                }
            },
            addPerformance: function (performance, skipTimelineUpdate) {
                this.queueCollection.add(new QueueItem({performance: performance}));
                this.fetchPerformances([performance.get('id')]);
                if (!skipTimelineUpdate) this.updateTimeline();
            },
            updateTimeline: function (options) {
                let ids = this._getPerformanceIds();
                this.timelinesView = this._showTimeline(_.extend({
                    sequence: ids,
                    readonly: true
                }, options || {}));
            },
            stop: function () {
                if (this.timelinesView) {
                    this.timelinesView.destroy();
                    this.timelinesView = null;
                }
            },
            clearClick: function () {
                this.stop();
                this.clearQueue();
                this.updateTimeline();
                this.ui.clearButton.blur();
            },
            clearQueue: function () {
                this.queueCollection.reset();
            },
            _getPerformanceIds: function () {
                let ids = [];

                this.queueCollection.each(function (item) {
                    let performance = item.get('performance');
                    if (performance instanceof Performance && performance.id)
                        ids.push(performance.id)
                });

                return ids;
            },
            _showTimeline: function (options) {
                let self = this,
                    playButtons = this.ui.queue.find('.app-play')

                this.editting = !options.readonly;

                if (this.editting)
                    playButtons.fadeOut();
                else
                    playButtons.fadeIn();

                this.timelinesView = new TimelinesView(_.extend({
                    performances: this.performances,
                    disableSaving: this.disableSaving,
                    layoutView: this
                }, options));

                this.listenTo(this.timelinesView, 'close', function () {
                    self.updateTimeline();
                });

                self.time = 0;
                this.listenTo(this.timelinesView, 'change:time', function (time) {
                    if (!this.editting) {
                        self.time = time;
                        let offset = 0;

                        self.queueCollection.find(function (item) {
                            offset += item.get('performance').getDuration();
                            let found = time < offset;

                            if (found) {
                                let el = self.ui.queueContainer.find('[data-model-cid=' + item.cid + ']')
                                if (!el.hasClass('active')) {
                                    self.ui.queueContainer.find('.app-performance').removeClass('active');
                                    el.addClass('active');
                                }
                            }

                            return found;
                        });
                    }
                });

                this.listenTo(this.timelinesView, 'idle', function () {
                    self.ui.queue.find('.app-performance').removeClass('active');
                });

                // show configuration UI
                this.getRegion('timeline').show(this.timelinesView);
                return this.timelinesView;
            }
        });
    });
