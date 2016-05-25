define(['marionette', 'tpl!./templates/performances.tpl', './performance', '../entities/performance', 'underscore',
        'typeahead'],
    function (Marionette, template, PerformanceView, Performance, _) {
        return Marionette.CompositeView.extend({
            template: template,
            childView: PerformanceView,
            childViewContainer: '.app-performances',
            ui: {
                newButton: '.app-new-performance-button',
                tabs: '.app-performance-group-tabs',
                container: '.app-performances'
            },
            events: {
                'click @ui.newButton': 'addNew'
            },
            collectionEvents: {
                'change:path add remove reset': 'updateTabs'
            },
            addNew: function () {
                var performance = new Performance({name: 'New performance'});
                performance.set({path: this.currentPath + '/' + performance.id})
                this.collection.add(performance);
                this.trigger('new', performance);
            },
            _insertAfter: function (childView) {
                var self = this;

                // add performance to the queue on click
                childView.on('click', function (data) {
                    self.options.queueView.addPerformance(data.model);
                });

                this.ui.newButton.before(childView.el);

                // hiding if not from current directory
                if (childView.model.get('path').split('/').slice(0, -1).join('/') != this.currentPath)
                    childView.$el.hide();
            },
            currentPath: '',
            updateTabs: function () {
                var self = this,
                    paths = _.compact(_.uniq(this.collection.pluck('path'))),
                    dirs = [];

                // create a list of all directories
                _.each(paths, function (path, i) {
                    path = path.split('/').slice(0, -1);
                    for (var i = 0; i < path.length; i++)
                        dirs.push(path.slice(0, i + 1).join('/'));
                });
                dirs = _.uniq(dirs);

                var depth = (this.currentPath == '') ? 0 : this.currentPath.split('/').length,
                    currentDirs = _.filter(dirs, function (dir) {
                        // filtering only dirs in current directory
                        dir = dir.split('/');
                        return dir.length == depth + 1 && dir.slice(0, -1).join('/') == self.currentPath;
                    }),
                    createTab = function (dir, name, disableClickEvent) {
                        if (!name) {
                            name = dir.split('/');
                            name = name[name.length - 1];
                        }

                        var el = $('<a>').attr('href', 'javascript:void(0)').html(name);
                        if (!disableClickEvent) {
                            el.click(function () {
                                $('.app-performance-button', this.$el).hide().filter('[data-path="' + dir + '"]').fadeIn();
                                self.currentPath = dir;
                                self.updateTabs();
                            });
                        }
                        return $('<li>').attr('data-path', dir).append(el);
                    };

                // clear tabs
                this.ui.tabs.html('');
                // adding tab for parent dir if available
                if (depth > 0) this.ui.tabs.append(createTab(this.currentPath.split('/').slice(0, -1).join('/'), '..'));

                _.each(currentDirs, function (dir) {
                    self.ui.tabs.append(createTab(dir));
                });
                this.ui.tabs.append(
                    createTab(this.currentPath, '/' + this.currentPath, true).addClass('app-current-path active'));
            }
        });
    });
