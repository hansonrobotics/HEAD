let bootbox = require('bootbox')

define(['marionette', 'backbone', './templates/layout.tpl', 'lib/regions/fade_in', 'lib/api', './performances',
        '../entities/performance_collection', '../entities/performance', './queue', './timelines', 'jquery',
        'underscore', '../entities/queue', '../entities/queue_item', 'select2', 'select2-css'],
    function(Marionette, Backbone, template, FadeInRegion, api, PerformancesView, PerformanceCollection, Performance,
             QueueView, TimelinesView, $, _, QueueCollection, QueueItem) {
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
                emptyNotice: '.app-empty-notice',
                saveChangesModal: '.app-save-changes-confirmation',
                saveChanges: '.app-save-changes',
                discardChanges: '.app-discard-changes'
            },
            events: {
                'click @ui.languageButton': 'changeLanguage',
                'click @ui.clearButton': 'clearClick',
                'click @ui.saveChanges': 'saveChanges',
                'click @ui.discardChanges': 'discardChanges',
                'hide.bs.modal @ui.saveChangesModal': 'saveChangesHide'
            },
            initialize: function(options) {
                this.mergeOptions(options, ['editing', 'autoplay', 'dir', 'nav', 'readonly', 'hideQueue', 'disableSaving', 'allowEdit',
                    'sequence'])
                this.performances = new PerformanceCollection()
                this.queueCollection = new QueueCollection()
            },
            onAttach: function() {
                // fluid by default
                this.setFluidContainer(this.fluid || typeof this.fluid === 'undefined')

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
                    })

                self.getRegion('queue').show(queueView)
                self.getRegion('performances').show(performancesView)

                this.listenTo(performancesView, 'new', function(p) {
                    this._showTimeline({
                        model: p,
                        readonly: false
                    })
                })

                this.performances.fetch({
                    reset: true,
                    success: function() {
                        if (!self.sequence)
                            self.showCurrent()
                    }
                })

                if (this.hideQueue)
                    this.ui.queueContainer.hide()

                if (this.sequence)
                    this.addSequence(this.sequence)

                if (this.readonly) {
                    this.runningPerformances = new Performance()
                    this.runningPerformances.enableSync(function(performances) {
                        self.clearQueue()
                        self.addSequence(_.map(performances, 'id'), true)
                    })
                }
            },
            onDestroy: function() {
                if (this.readonly) this.runningPerformances.disableSync()
            },
            changeLanguage: function(e) {
                var language = $(e.target).data('lang')

                this.ui.languageButton.removeClass('active')
                $(e.target).addClass('active')

                api.setRobotLang(language)
            },
            setFluidContainer: function(enable) {
                if (enable)
                    this.ui.container.removeClass('container').addClass('container-fluid')
                else
                    this.ui.container.removeClass('container-fluid').addClass('container')
            },
            addSequence: function(sequence, skipTimelineUpdate) {
                let self = this

                if (!_.isEqual(this._getPerformanceIds(), sequence)) {
                    if (sequence instanceof Array && this.performances) {
                        _.each(sequence, function(id) {
                            let model = self.performances.get(id)
                            if (model) self.addPerformance(model, true)
                        })
                    }

                    if (!skipTimelineUpdate) this.updateTimeline()
                }
            },
            showCurrent: function() {
                let self = this,
                    current = new Performance()

                current.fetchCurrent({
                    success: function(response) {
                        self._showTimeline({
                            model: current,
                            running: response['running'],
                            paused: response['paused'],
                            current_time: response['current_time'],
                            readonly: true
                        })

                        let ids = _.map(response['performances'], 'id')
                        self.addSequence(ids, true)
                        self.fetchPerformances(ids)
                    }
                })
            },
            fetchPerformances: function(ids) {
                let self = this

                // fetch full info for performances in the sequence
                _.each(ids, function(id) {
                    let p = self.performances.get(id)
                    if (p && p.nodes.isEmpty()) p.fetch()
                })
            },
            editCurrent: function() {
                let item = this.queueCollection.findItemByTime(this.time)
                if (item) this.editItem(item)
            },
            editPrevious: function(item) {
                let self = this
                this.changeCheck(function() {
                    let i = self.queueCollection.findIndex(item)
                    if (i > 0) self.editItem(self.queueCollection.at(i - 1))
                })
            },
            editNext: function(item) {
                let self = this
                this.changeCheck(function() {
                    let i = self.queueCollection.findIndex(item)
                    if (i >= 0 && i < self.queueCollection.length - 1) self.editItem(self.queueCollection.at(i + 1))
                });
            },
            editItem: function(item) {
                if (!this.timelinesView || this.timelinesView.readonly || item.get('performance') !== this.timelinesView.model) {
                    let self = this
                    this.changeCheck(function() {
                        self.highlight(item)
                        self._showTimeline({
                            model: item.get('performance'),
                            queueItem: item,
                            readonly: false
                        })
                    })
                }
            },
            play: function(item) {
                if (!this.timelinesView || !this.timelinesView.readonly)
                    this.updateTimeline()
                this.timelinesView.run(item.getStartTime())
            },
            remove: function(item) {
                this.queueCollection.remove(item)
                if (!this.editting)
                    this.updateTimeline()
            },
            setTime: function(item) {
                if (this.editting) {
                    this.editItem(item)
                } else {
                    if (!this.timelinesView) this.updateTimeline()
                    this.timelinesView.moveIndicator(item.getStartTime())
                }
            },
            addPerformance: function(performance, skipTimelineUpdate) {
                let item = new QueueItem({performance: performance}, {silent: skipTimelineUpdate})
                this.queueCollection.add(item)
                this.fetchPerformances([performance.get('id')])
                if (!skipTimelineUpdate) this.updateTimeline()
                return item
            },
            updateTimeline: function(options) {
                let ids = this._getPerformanceIds()
                this.timelinesView = this._showTimeline(_.extend({
                    sequence: ids,
                    readonly: true
                }, options || {}))
            },
            stop: function() {
                if (this.timelinesView) {
                    this.timelinesView.destroy()
                    this.timelinesView = null
                }
            },
            clearClick: function() {
                this.stop()
                this.clearQueue()
                this.updateTimeline()
                this.ui.clearButton.blur()
            },
            clearQueue: function() {
                this.queueCollection.reset()
            },
            highlight: function(item) {
                let el = this.ui.queueContainer.find('[data-model-cid=' + item.cid + ']')
                if (!el.hasClass('active')) {
                    this.ui.queueContainer.find('.app-performance').removeClass('active')
                    el.addClass('active')
                }
            },
            changeCheck: function(callback, cancelCallback) {
                if (this.timelinesView && !this.timelinesView.isDestroyed() && this.timelinesView.changed) {
                    this.changeCheckModalCancelled = true
                    this.changeCheckCallback = callback
                    this.changeCheckCancelCallback = cancelCallback
                    this.ui.saveChangesModal.modal()
                } else
                    callback(true);
            },
            saveChanges: function() {
                this.timelinesView.model.save()
                this.changeCheckCallback(true)
                this.changeCheckModalCancelled = false
                this.ui.saveChangesModal.modal('hide')
            },
            discardChanges: function() {
                this.timelinesView.revert()
                this.changeCheckCallback(false)
                this.changeCheckModalCancelled = false
                this.ui.saveChangesModal.modal('hide')
            },
            saveChangesHide: function() {
                if (this.changeCheckModalCancelled && typeof this.changeCheckCancelCallback === 'function')
                    this.changeCheckCancelCallback()
            },
            _getPerformanceIds: function() {
                let ids = []

                this.queueCollection.each(function(item) {
                    let performance = item.get('performance')
                    if (performance instanceof Performance && performance.id)
                        ids.push(performance.id)
                })

                return ids
            },
            _showTimeline: function(options) {
                let self = this,
                    playButtons = this.ui.queue.find('.app-play')

                this.editting = !options.readonly

                if (this.editting)
                    playButtons.fadeOut()
                else
                    playButtons.fadeIn()

                this.timelinesView = new TimelinesView(_.extend({
                    performances: this.performances,
                    disableSaving: this.disableSaving,
                    layoutView: this,
                    queue: this.queueCollection,
                    allowEdit: this.allowEdit
                }, options))

                this.listenTo(this.timelinesView, 'close', function() {
                    self.updateTimeline()
                })

                self.time = 0
                this.listenTo(this.timelinesView, 'change:time', function(time) {
                    if (!this.editting) {
                        self.time = time
                        let offset = 0

                        self.queueCollection.find(function(item) {
                            offset += item.get('performance').getDuration()
                            let found = time < offset

                            if (found) self.highlight(item)

                            return found
                        })
                    }
                })

                this.listenTo(this.timelinesView, 'idle', function() {
                    self.ui.queue.find('.app-performance').removeClass('active')
                })

                // show configuration UI
                this.getRegion('timeline').show(this.timelinesView)
                return this.timelinesView
            }
        })
    })
