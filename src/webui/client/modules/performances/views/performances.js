define(['application', 'marionette', 'backbone', './templates/performances.tpl', './performance', '../entities/performance', 'underscore',
        'jquery', 'bootbox', 'lib/api', './settings', 'path', 'natural-sort', 'typeahead', 'jquery-ui'],
    function(app, Marionette, Backbone, template, PerformanceView, Performance, _, $, bootbox, api, SettingsView, path,
             naturalSort) {
        return Marionette.CompositeView.extend({
            template: template,
            childView: PerformanceView,
            childViewContainer: '.app-performances',
            ui: {
                newButton: '.app-new-performance-button',
                addAllButton: '.app-add-all-button',
                tabs: '.app-performance-group-tabs',
                container: '.app-performances',
                dirHeader: '.app-performance-dir-header'
            },
            events: {
                'click @ui.newButton': 'addNew',
                'click @ui.addAllButton': 'addAll'
            },
            collectionEvents: {
                'change:path reset': 'updateTabs'
            },
            initialize: function(options) {
                this.mergeOptions(options, ['readonly', 'nav', 'autoplay', 'layoutView', 'dir'])
            },
            onRender: function() {
                let self = this,
                    deactivate = function() {
                        $('.app-current-path', self.ui.tabs).removeClass('highlight')
                    }

                if (this.autoplay) this.ui.addAllButton.get(0).lastChild.nodeValue = ' Play All'
                if (this.readonly) this.ui.newButton.hide()

                this.ui.container.droppable({
                    accept: '.app-performance-button',
                    tolerance: 'pointer',
                    hoverClass: 'active',
                    over: function() {
                        $('.app-current-path', self.ui.tabs).addClass('highlight')
                    },
                    deactivate: deactivate,
                    out: deactivate,
                    drop: function(event, ui) {
                        let view = self.children.findByCid(ui.draggable.data('cid'))
                        if (view && self.currentPath != view.model.get('path')) {
                            view.model.set({'path': self.currentPath, ignore_nodes: true})
                            view.model.save()
                        }
                        self.updateVisiblePerformances(self.currentPath)
                    }
                })

                if (this.dir) this.switchDir(this.dir)
                else this.updateTabs()
            },
            childViewOptions: function() {
                return this.options
            },
            addNew: function() {
                let performances = new Backbone.Collection(this.collection.where({path: this.currentPath})),
                    names = performances.pluck('name')

                this.newestPerformance = new Performance({
                    name: this.getNextName(names, path.basename(this.currentPath) || 'Performance'),
                    path: this.currentPath
                })

                this.collection.add(this.newestPerformance);
                this.trigger('new', this.newestPerformance);
                this.ui.addAllButton.fadeIn();
            },
            getNextName: function(names, defaultPrefix) {
                let self = this,
                    prefix,
                    number = null

                // use the prefix of the last created performance in this dir
                if (this.newestPerformance && this.newestPerformance.get('name')) {
                    prefix = this.newestPerformance.get('name')
                    // set an empty prefix if it's numeric
                    if ($.isNumeric(prefix)) prefix = ' '
                    else prefix = prefix.replace(/\d+$/, '')
                }

                if (prefix) {
                    names = _.filter(names, function(name) {
                        if (prefix === ' ') {
                            return $.isNumeric(name)
                        } else {
                            let regex = new RegExp('^' + self.escape(prefix) + '\d*', 'i')
                            return regex.test(name)
                        }
                    })
                }

                naturalSort.insensitive = true
                names = names.sort(naturalSort)

                if (names.length) {
                    let name = names[names.length - 1]
                    prefix = name.replace(/\d*$/, '') || ' '
                    number = name.replace(prefix, '') || 0
                }

                prefix = prefix || defaultPrefix

                if (number !== null)
                    return prefix.trim() + (parseInt(number) + 1)
                else
                    return prefix
            },
            zeroPad: function(num, places) {
                let zero = places - num.toString().length + 1
                return Array(+(zero > 0 && zero)).join("0") + num
            },
            escape: function(str) {
                return str.replace(/[\-\[\]{}()*+?.,\\\^$|#\s]/g, "\\$&")
            },
            addAll: function() {
                let self = this,
                    added = false

                if (this.autoplay) self.layoutView.clearQueue()

                this.collection.each(function(performance) {
                    if ((performance.get('path') || '') == self.currentPath) {
                        self.layoutView.addPerformance(performance, true)
                        added = true
                    }
                })

                self.layoutView.updateTimeline({autoplay: this.autoplay})
            },
            attachHtml: function(collectionView, childView) {
                let self = this

                // add performance to the queue on click
                childView.on('click', function(data) {
                    if (self.autoplay) {
                        self.layoutView.clearQueue()
                        self.layoutView.addPerformance(data.model, true)
                        self.layoutView.updateTimeline({autoplay: true})
                    } else {
                        let timelinesView = self.layoutView.timelinesView
                        self.layoutView.addPerformance(data.model, !timelinesView || timelinesView.changed)
                    }

                    if (!data.model.nodes.length)
                        data.model.fetch()
                })

                this.ui.newButton.before(childView.el)

                // hiding if not from current directory
                if ((childView.model.get('path') || '') == this.currentPath)
                    childView.$el.show()
                else
                    childView.$el.hide()
            },
            currentPath: '',
            createdDirs: [],
            updateTabs: function() {
                let self = this,
                    depth = (this.currentPath == '') ? 0 : this.currentPath.split('/').length,
                    currentDirs = this.getCurrentDirs()

                // clear tabs
                this.ui.tabs.html('')

                // adding tab for parent dir if available
                if (depth > 0) this.ui.tabs.append(this.createTab(this.getParentPath(this.currentPath), '..'))

                _.each(currentDirs, function(dir) {
                    self.ui.tabs.append(self.createTab(dir))
                })

                let addNewTab = self.createTab(this.currentPath, $('<span>').addClass('glyphicon glyphicon-plus')
                    .attr('aria-hidden', 'true'), true)

                addNewTab.click(function (e, ui) {
                    addNewTab.hide()
                    let input = $('<input>').addClass('form-control input-sm'),
                        okButton = $('<button/>', {class: 'btn-sm btn btn-primary'}).html('OK'),
                        container = $('<div/>', {class: 'form-inline'}).append($('<div/>', {class: 'form-group'})
                            .append(input)).append(okButton),
                        newTab = $('<li>').addClass('app-new-dir').html(container)
                    input.focusout(function () {
                        setTimeout(function () {
                            if (!input.is(':focus')) {
                                newTab.fadeOut(300, function () {
                                    addNewTab.fadeIn(300);
                                    $(newTab).remove();
                                });
                            }
                        }, 500)
                    });

                    okButton.click(function () {
                        let dir = $(input).val().trim()
                        if (dir) {
                            dir = self.joinPaths(self.currentPath, dir)
                            self.createdDirs.push(dir)
                            self.switchDir(dir)
                        } else {
                            input.focus()
                        }
                    });

                    $(this).before(newTab)
                    input.focus()
                });

                if (!this.readonly)
                    this.ui.tabs.append(addNewTab)

                this.ui.dirHeader.html('/' + this.currentPath)

                if (!this.readonly)
                    self.ui.tabs.append(this.createTab(this.currentPath, 'Settings', true).addClass('pull-right').click(function() {
                        self.showSettings()
                    }))
            },
            getCurrentDirs: function() {
                let self = this,
                    paths = _.compact(_.uniq(this.collection.pluck('path'))),
                    dirs = []

                // create a list of all directories
                _.each(paths, function(path) {
                    path = path.split('/')
                    for (let i = 0; i < path.length; i++)
                        dirs.push(path.slice(0, i + 1).join('/'));
                })

                dirs = _.filter(_.uniq(_.union(dirs, this.createdDirs)), function(dir) {
                    return self.getParentPath(dir) == self.currentPath
                })

                naturalSort.insensitive = true
                return dirs.sort(naturalSort)
            },
            showSettings: function() {
                let settingsView = new SettingsView({
                    path: this.currentPath
                })

                bootbox.dialog({
                    title: 'Performance Settings',
                    message: settingsView.$el,
                    onEscape: true,
                    backdrop: true,
                    size: 'large'
                }).on("shown.bs.modal", function() {
                    settingsView.render()
                })
            },
            createTab: function(dir, content, disableEvents) {
                let self = this

                if (!content) {
                    content = dir.split('/')
                    content = content[content.length - 1]
                }

                let timeout = null,
                    el = $('<a>').attr('href', 'javascript:void(0)').html(content)

                if (this.nav)
                    el.attr('href', path.join('/#/performances', dir))

                if (!disableEvents)
                    el.click(function() {
                        self.switchDir(dir)
                    }).droppable({
                        accept: '.app-performance-button',
                        tolerance: 'pointer',
                        over: function() {
                            $(this).parent().addClass('active')
                            timeout = setTimeout(function() {
                                self.switchDir(dir)
                            }, 600)
                        },
                        out: function() {
                            $(this).parent().removeClass('active')
                            clearTimeout(timeout)
                        }
                    })

                return $('<li>').attr('data-path', dir).append(el)
            },
            switchDir: function(dir) {
                if (this.nav) {
                    let url = path.join('/performances', dir)
                    if (url !== Backbone.history.getHash()) {
                        app.skipChangeCheck = true
                        Backbone.history.navigate('#' + url)
                    }
                }

                this.updateVisiblePerformances(dir)
                this.currentPath = dir
                this.collection.currentPath = dir
                // reset newest performance so that it's name isn't used for naming
                this.newestPerformance = null
                this.updateTabs()
            },
            updateVisiblePerformances: function (dir) {
                let performances = $('.app-performance-button:not(.ui-draggable-dragging)', this.$el).hide().filter('[data-path="' + dir + '"]')
                if (performances.length) {
                    performances.fadeIn()
                    this.ui.addAllButton.fadeIn()
                } else {
                    this.ui.addAllButton.hide()
                }
            },
            joinPaths: function(path1, path2) {
                return _.compact(_.union((path1 || '').split('/'), (path2 || '').split('/'))).join('/')
            },
            getParentPath: function(path) {
                return _.compact((path || '').split('/').slice(0, -1)).join('/')
            }
        })
    })
