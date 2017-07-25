let bootbox = require('bootbox')

define(['application', 'marionette', 'backbone', './templates/layout.tpl', 'lib/regions/fade_in', 'lib/api', './performances',
        '../entities/performance_collection', '../entities/performance', './queue', './timelines', 'jquery',
        'underscore', '../entities/queue', '../entities/queue_item', 'mousetrap', 'select2', 'select2-css'],
    function(app, Marionette, Backbone, template, FadeInRegion, api, PerformancesView, PerformanceCollection, Performance,
             QueueView, TimelinesView, $, _, QueueCollection, QueueItem, Mousetrap) {
        return Marionette.View.extend({
            template: template,
            className: 'app-performances-page',
            regions: {
                performances: {
                    el: '.app-performances-region',
                    regionClass: FadeInRegion
                },
                queue: {
                    el: '.app-queue-region'
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
                addNewButton: '.app-add-new-button',
                emptyNotice: '.app-empty-notice',
                saveChangesModal: '.app-save-changes-confirmation',
                saveChanges: '.app-save-changes',
                discardChanges: '.app-discard-changes'
            },
            events: {
                'click @ui.languageButton': 'changeLanguage',
                'click @ui.addNewButton': 'addNewTimeline',
                'click @ui.saveChanges': 'saveChanges',
                'click @ui.discardChanges': 'discardChanges',
                'hidden.bs.modal @ui.saveChangesModal': 'saveChangesHide'
            },
            initialize: function(options) {
                this.mergeOptions(options, ['editing', 'autoplay', 'dir', 'nav', 'readonly', 'hideQueue',
                    'disableSaving', 'allowEdit', 'queueHeight', 'urlPrefix'])
                this.performances = new PerformanceCollection()
                this.queueCollection = new QueueCollection()
            },
            onAttach: function() {
                let self = this

                this.ui.queueContainer.hide()

                // set change check callback
                app.changeCheck = _.bind(this.changeCheck, this)

                // fluid by default
                this.setFluidContainer(this.fluid || typeof this.fluid === 'undefined')

                this.queueView = new QueueView({
                    collection: this.queueCollection,
                    readonly: this.readonly,
                    layoutView: this,
                    height: this.queueHeight
                })
                this.performancesView = new PerformancesView({
                    collection: self.performances,
                    layoutView: this,
                    readonly: this.readonly,
                    autoplay: this.autoplay,
                    dir: this.dir,
                    nav: this.nav,
                    urlPrefix: this.urlPrefix
                })

                this.listenTo(this.queueView, 'reordered', this.updateTimelineOrder)

                self.getRegion('queue').show(this.queueView)
                self.getRegion('performances').show(this.performancesView)

                this.listenTo(this.performancesView, 'new', function(p) {
                    self.performances.add(p)
                    self.performancesView.navigate(p.get('id'))
                    self.editItem(self.queueCollection.first())
                })

                this.syncCallback = this.syncCallback.bind(this)
                this.syncedPerformance = new Performance()
                this.syncedPerformance.enableSync(this.syncCallback)

                this.performances.fetch({
                    reset: true,
                    success: function() {
                        if (!self.performances.get(self.dir))
                            self.showCurrent()
                    }
                })

                this.registerShortcuts()
            },
            registerShortcuts: function() {
                let self = this
                Mousetrap.bind('y', function() {
                    if (self.editing)
                        self.ui.saveChanges.click()
                })

                Mousetrap.bind('n', function() {
                    if (self.editing)
                        self.ui.discardChanges.click()
                })

                Mousetrap.bind('ctrl+m', function(e) {
                    if (self.editing)
                        self.addNewTimeline()
                })
            },
            unregisterShortcuts: function() {
                Mousetrap.unbind('y')
                Mousetrap.unbind('n')
                Mousetrap.unbind('ctrl+m')
            },
            onDestroy: function() {
                this.syncedPerformance.disableSync()
                app.changeCheck = null
                this.unregisterShortcuts()
            },
            setCurrentPerformance: function(p, options) {
                options = options || {}

                if (this.currentPerformance)
                    this.stopListening(this.currentPerformance)

                if (p) {
                    this.currentPerformance = p
                    this.refreshCurrentPerformance(options)
                } else if (this.currentPerformance) {
                    Performance.unload()
                    this.currentPerformance = null
                    this.destroyTimeline()
                    this.queueCollection.reset()
                }
            },
            syncCallback: function(p) {
                if (!this.timelinesView || this.timelinesView.readonly) {
                    if (p) {
                        if (!this.currentPerformance || p.id !== this.currentPerformance.id) {
                            this.setCurrentPerformance(new Performance(p), {
                                skipLoading: true,
                                readonly: true
                            })

                            if (p.id) this.performancesView.navigate(p.id)
                        }
                    } else {
                        this.setCurrentPerformance(null)
                        this.performancesView.backToDir()
                    }
                }
            },
            destroyTimeline: function() {
                if (this.timelinesView) {
                    this.timelinesView.destroy()
                    this.ui.queueContainer.hide()
                }
            },
            refreshCurrentPerformance: function(options) {
                options = options || {}
                let self = this
                if (!options.skipLoading)
                    this.currentPerformance.nodes.reset()
                this.clearQueue()
                this.updateTimelineQueue(new PerformanceCollection(this.currentPerformance.get('timelines')))
                this.listenTo(this.currentPerformance, 'change:timelines', function() {
                    self.updateTimelineQueue(new PerformanceCollection(this.currentPerformance.get('timelines')))
                    self.toggleQueue()
                })
                this.toggleQueue()
                this.updateTimeline(options)
            },
            toggleQueue: function() {
                if (this.currentPerformance.get('timelines') && !this.hideQueue)
                    this.ui.queueContainer.fadeIn()
                else
                    this.ui.queueContainer.fadeOut()
            },
            updateTimelineQueue: function(timelines) {
                for (let i = 0; i < timelines.length; i++) {
                    let item = this.queueCollection.at(i),
                        timeline = timelines.at(i)

                    if (item) {
                        item.get('performance').set(timeline.toJSON())

                    } else
                        this.queueCollection.add(new QueueItem({performance: timeline}))
                }

                if (this.editing) {
                    this.timelinesView.changed = false
                    this.highlight(this.timelinesView.model)
                }
            },
            updateTimelineOrder: function(success) {
                if (this.currentPerformance) {
                    let self = this,
                        timelines = []

                    for (let i = 0; i < this.queueCollection.length; i++) {
                        let p = this.queueCollection.at(i).get('performance')
                        p.set('name', (i + 1).toString()).unset('previous_id')
                        timelines.push(p.toJSON())
                    }

                    this.currentPerformance.save({'timelines': timelines}, {
                        success: function() {
                            if (!self.editing)
                                self.refreshCurrentPerformance()

                            if (success) success()
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
                let self = this

                this.syncedPerformance.fetchCurrent({
                    success: function(response) {
                        if (self.syncedPerformance.get('timelines') || self.readonly) {
                            self.setCurrentPerformance(self.syncedPerformance, {
                                running: response['running'],
                                paused: response['paused'],
                                current_time: response['current_time'],
                                readonly: true,
                                skipLoading: true
                            })

                            if (self.syncedPerformance.id)
                                self.performancesView.navigate(self.syncedPerformance.id)

                            if (!response.performance) {
                                self.destroyTimeline()
                            }
                        } else {
                            self.performancesView.back(self.syncedPerformance.id)
                        }
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
            getCurrentItem: function() {
                return this.queueCollection.findItemByTime(this.time)
            },
            editCurrent: function() {
                let item = this.getCurrentItem()
                if (item) this.editItem(item)
                else this.addNewTimeline()
            },
            editPrevious: function(item) {
                let i = this.queueCollection.findIndex(item)
                if (i > 0) this.editItem(this.queueCollection.at(i - 1))
            },
            editNext: function(item) {
                let i = this.queueCollection.findIndex(item)
                if (i >= 0 && i < this.queueCollection.length - 1) this.editItem(this.queueCollection.at(i + 1))
            },
            movePrevious: function() {
                let i = this.queueCollection.findIndex(this.getCurrentItem())
                if (i >= 0)
                    this.timelinesView.moveIndicator(this.queueCollection.at(Math.max(0, i - 1)).getStartTime())
            },
            moveNext: function(item) {
                let i = this.queueCollection.findIndex(this.getCurrentItem())
                if (i >= 0 && i <= this.queueCollection.length - 1)
                    this.timelinesView.moveIndicator(
                        this.queueCollection.at(Math.min(this.queueCollection.length - 1, i + 1)).getStartTime())
            },
            editItem: function(item, options) {
                let self = this
                options = options || {}
                if (!this.timelinesView || this.timelinesView.readonly || item !== this.timelinesView.queueItem) {
                    this.changeCheck(function() {
                        self.highlight(item)
                        self._showTimeline(_.extend(options, {
                            model: item.get('performance'),
                            queueItem: item,
                            readonly: false
                        }))
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
                bootbox.confirm('Are you sure?', function(result) {
                    if (result) {
                        self.queueCollection.remove(item)
                        item.get('performance').destroy()
                        self.updateTimelineOrder()

                    }
                })
            },
            setItemTime: function(item) {
                if (this.editing) {
                    this.editItem(item)
                } else {
                    if (!this.timelinesView) this.refreshCurrentPerformance()
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
            addNewTimeline: function() {
                function saveTimeline() {
                    timelines.push(performance)
                    performance.save({}, {
                        success: function() {
                            self.editItem(item)
                            self.queueCollection.add(item)
                        }
                    })
                }

                let current = this.currentPerformance,
                    timelines = current.get('timelines'),
                    name = (timelines.length + 1).toString(),
                    self = this,
                    performance = new Performance({
                        path: current.get('id'),
                        name: name
                    }),
                    item = new QueueItem({
                        performance: performance
                    })

                if (_.findIndex(timelines, {name: name}) > -1) {
                    this.updateTimelineOrder(function() {
                        saveTimeline()
                    })
                } else {
                    saveTimeline()
                }
            },
            clearQueue: function() {
                this.queueCollection.reset()
            },
            highlight: function(item) {
                if (item instanceof Performance)
                    item = this.queueCollection.find({performance: item})

                if (item instanceof QueueCollection) {
                    let el = this.ui.queueContainer.find('[data-model-cid=' + item.cid + ']')

                    if (!el.hasClass('active')) {
                        this.ui.queueContainer.find('.app-performance').removeClass('active')
                        el.addClass('active')
                    }
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
                let self = this

                this.editing = !options.readonly
                if (this.editing)
                    this.queueView.disablePlay()
                else
                    this.queueView.enablePlay()

                if (this.timelinesView) {
                    this.stopListening(this.timelinesView)
                    this.timelinesView.destroy()
                }
                this.timelinesView = new TimelinesView(_.extend({
                    performances: this.performances,
                    disableSaving: this.disableSaving,
                    layoutView: this,
                    queue: this.queueCollection,
                    allowEdit: this.allowEdit
                }, options))

                this.listenTo(this.timelinesView, 'edit:done', function() {
                    this.refreshCurrentPerformance()
                })

                this.listenTo(this.timelinesView, 'change:time', function(time) {
                    if (!this.editing) {
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
