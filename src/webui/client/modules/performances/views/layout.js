let bootbox = require('bootbox')

define(['application', 'marionette', 'backbone', './templates/layout.tpl', 'lib/regions/fade_in', 'lib/api', './performances',
        '../entities/performance_collection', '../entities/performance', './queue', './timelines', 'jquery',
        'underscore', '../entities/queue', '../entities/queue_item', 'select2', 'select2-css'],
    function(app, Marionette, Backbone, template, FadeInRegion, api, PerformancesView, PerformanceCollection, Performance,
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
                'hidden.bs.modal @ui.saveChangesModal': 'saveChangesHide'
            },
            initialize: function(options) {
                this.mergeOptions(options, ['editing', 'autoplay', 'dir', 'nav', 'readonly', 'hideQueue', 'disableSaving', 'allowEdit'])
                this.performances = new PerformanceCollection()
                this.queueCollection = new QueueCollection()
            },
            onAttach: function() {
                let self = this

                // set change check callback
                app.changeCheck = _.bind(this.changeCheck, this)

                // fluid by default
                this.setFluidContainer(this.fluid || typeof this.fluid === 'undefined')

                this.queueView = new QueueView({
                    collection: this.queueCollection,
                    readonly: this.readonly,
                    layoutView: this
                })
                this.performancesView = new PerformancesView({
                    collection: self.performances,
                    layoutView: this,
                    readonly: this.readonly,
                    autoplay: this.autoplay,
                    dir: this.dir,
                    nav: this.nav
                })

                this.listenTo(this.queueView, 'reordered', this.updateTimelineOrder)

                self.getRegion('queue').show(this.queueView)
                self.getRegion('performances').show(this.performancesView)

                this.listenTo(this.performancesView, 'new', function(p) {
                    this._showTimeline({
                        model: p,
                        readonly: false
                    })
                })

                this.listenTo(this.performancesView, 'selected', function(p) {
                    this.setCurrentPerformance(p)
                })

                this.performances.fetch({
                    reset: true,
                    success: function() {
                        self.showCurrent()
                    }
                })

                if (this.hideQueue)
                    this.ui.queueContainer.hide()

                if (this.readonly) {
                    let p = new Performance()
                    self.setCurrentPerformance(p)
                    p.enableSync()
                }
            },
            onDestroy: function() {
                app.changeCheck = null
                if (this.currentPerformance) this.currentPerformance.disableSync()
            },
            setCurrentPerformance: function(p, options) {
                if (this.currentPerformance) {
                    this.currentPerformance.disableSync()
                    this.stopListening(this.currentPerformance)
                }

                this.currentPerformance = p
                this.refreshCurrentPerformance(options)
            },
            refreshCurrentPerformance: function(options) {
                let self = this
                this.currentPerformance.nodes.reset()
                this.setTimelineQueue(new PerformanceCollection(this.currentPerformance.get('timelines')))
                this.listenTo(this.currentPerformance, 'change:timelines', function() {
                    self.setTimelineQueue(new PerformanceCollection(this.currentPerformance.get('timelines')))
                })
                this.updateTimeline(options)
            },
            setTimelineQueue: function(timelines) {
                let items = []
                this.queueCollection.reset()
                timelines.each(function(timeline) {
                    items.push(new QueueItem({performance: timeline}))
                })
                this.queueCollection.set(items)
            },
            updateTimelineOrder: function() {
                if (this.currentPerformance) {
                    let self = this,
                        timelines = new PerformanceCollection()
                    this.queueCollection.each(function(item, i) {
                        let p = item.get('performance')
                        p.set('name', (i + 1).toString())
                        timelines.push(p)
                    })

                    console.log('save')
                    this.currentPerformance.save({'timelines': timelines.toJSON()}, {
                        success: function(r) {
                            console.log(r)
                            self.refreshCurrentPerformance()
                        }
                    })
                }
            },
            changeLanguage: function(e) {
                let language = $(e.target).data('lang')

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
            showCurrent: function() {
                let self = this,
                    current = new Performance()

                current.fetchCurrent({
                    success: function(response) {
                        self.setCurrentPerformance(current, {
                            running: response['running'],
                            paused: response['paused'],
                            current_time: response['current_time'],
                            readonly: true
                        })
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
                let i = this.queueCollection.findIndex(item)
                if (i > 0) this.editItem(this.queueCollection.at(i - 1))
            },
            editNext: function(item) {
                let i = this.queueCollection.findIndex(item)
                if (i >= 0 && i < this.queueCollection.length - 1) this.editItem(this.queueCollection.at(i + 1))
            },
            editItem: function(item) {
                let self = this
                if (!this.timelinesView || this.timelinesView.readonly || item !== this.timelinesView.queueItem) {
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
                let self = this
                bootbox.confirm("Are you sure?", function(result) {
                    self.queueCollection.remove(item)
                    item.get('performance').destroy()
                    if (!self.editting)
                        self.refreshCurrentPerformance()
                })
            },
            setItemTime: function(item) {
                if (this.editting) {
                    this.editItem(item)
                } else {
                    if (!this.timelinesView) this.updateTimeline()
                    this.timelinesView.moveIndicator(item.getStartTime())
                }
            },
            updateTimeline: function(options) {
                this.timelinesView = this._showTimeline(_.extend({
                    model: this.currentPerformance,
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
                let self = this
                bootbox.confirm("Are you sure?", function(result) {
                    self.stop()
                    self.ui.clearButton.blur()
                    self.currentPerformance.save({'timelines': []}, {
                        success: function() {
                            self.refreshCurrentPerformance()
                        }
                    })
                })
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
                if (!this.isDestroyed() && this.timelinesView && !this.timelinesView.isDestroyed() && this.timelinesView.changed) {
                    this.changeCheckResult = null
                    this.changeCheckCallback = callback
                    this.changeCheckCancelCallback = cancelCallback
                    this.ui.saveChangesModal.modal()
                } else
                    callback(true)
            },
            saveChanges: function() {
                this.timelinesView.model.save()
                this.changeCheckResult = true
                this.ui.saveChangesModal.modal('hide')
            },
            discardChanges: function() {
                this.timelinesView.revert()
                this.changeCheckResult = false
                this.ui.saveChangesModal.modal('hide')
            },
            saveChangesHide: function() {
                if (this.changeCheckResult === null) {
                    if (typeof this.changeCheckCancelCallback === 'function')
                        this.changeCheckCancelCallback()
                } else
                    this.changeCheckCallback(this.changeCheckResult)
            },
            time: 0,
            _showTimeline: function(options) {
                let self = this,
                    playButtons = this.ui.queue.find('.app-play')

                this.editting = !options.readonly
                if (this.editting)
                    this.queueView.disablePlay()
                else
                    this.queueView.enablePlay()

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
                    self.refreshCurrentPerformance()
                })

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
