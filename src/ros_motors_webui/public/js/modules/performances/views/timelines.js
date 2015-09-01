define(['application', 'tpl!./templates/timelines.tpl', './timeline', './node', 'scrollbar'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Timelines = Marionette.CompositeView.extend({
            template: template,
            childView: App.Performances.Views.Timeline,
            childViewContainer: '.app-timelines',
            config: {
                pxPerSec: 70
            },
            ui: {
                timelines: '.app-timelines',
                addButton: '.app-add-timeline-button',
                removeButton: '.app-remove-timeline-button',
                selectAll: '.app-timeline-select-all input',
                nodes: '.app-nodes .app-node',
                nodesContainer: '.app-nodes',
                nodeSettings: '.app-node-settings',
                scrollContainer: '.app-scroll-container',
                performanceName: '.app-performance-name',
                saveButton: '.app-save-button'
            },
            events: {
                'click @ui.addButton': 'addTimeline',
                'click @ui.removeButton': 'removeSelected',
                'click @ui.selectAll': 'selectAll',
                'click @ui.nodes': 'nodeClicked',
                'click @ui.saveButton': 'saveClicked',
                'keyup @ui.performanceName': 'setPerformanceName'
            },
            onShow: function () {
                this.addTimeline();
                $(this.ui.scrollContainer).scrollbar({});
                var self = this;
                this.model.get('nodes').each(function (node) {
                    self.createNodeEl(node);
                });

                this.arrangeNodes();
                this.model.get('nodes').bind('add', this.addNode, this);
            },
            onDestroy: function () {
                this.model.get('nodes').unbind('add', this.addNode, this);
            },
            nodeClicked: function (e) {
                var node = new App.Performances.Entities.Node({
                    name: $(e.target).data('name'),
                    offset: 0,
                    duration: 1,
                    text: ""
                });

                this.model.get('nodes').add(node);
                this.showNodeSettings(node);
            },
            createNodeEl: function (node) {
                var self = this,
                    el = $('.app-node[data-name="' + node.get('name') + '"]', this.ui.nodesContainer).clone().get(0);

                // show node settings on click
                $(el).on('click', function () {
                    self.showNodeSettings(node);
                });

                var updateNodeMetrics = function (model) {
                    $(model.get('el')).animate({
                        left: model.get('offset') * self.config.pxPerSec,
                        width: model.get('duration') * self.config.pxPerSec
                    })
                };

                node.set('el', el);
                node.on('change', function (model) {
                    if (typeof model.changed['offset'] != 'undefined' ||
                        typeof model.changed['duration'] != 'undefined'
                    ) {
                        updateNodeMetrics(model);
                        self.arrangeNodes();
                    }
                });

                updateNodeMetrics(node);
            },
            addNode: function (node) {
                this.createNodeEl(node);
                this.arrangeNodes();
            },
            showNodeSettings: function (node) {
                var self = this,
                    oldView = null;

                if (typeof this.nodeView != 'undefined')
                    oldView = this.nodeView;

                this.nodeView = new Views.Node({model: node});
                this.nodeView.render();

                this.ui.nodeSettings.slideUp(function () {
                    self.ui.nodeSettings.html(self.nodeView.el).hide().slideDown();
                    if (oldView)
                        oldView.destroy();
                });

                $(node.get('el')).addClass('active');
                this.nodeView.on('destroy', function () {
                    $(node.get('el')).removeClass('active');
                });
            },
            arrangeNodes: function () {
                var self = this,
                    available = false;

                this.model.get('nodes').each(function (node) {
                    var begin = node.get('offset'),
                        end = begin + node.get('duration');

                    self.children.every(function (view) {
                        available = true;

                        _.every($('.app-node', view.ui.nodes), function (el) {
                            var model = self.model.get('nodes').findWhere({'el': el});

                            if (model && model != node) {
                                var compareBegin = model.get('offset'),
                                    compareEnd = compareBegin + model.get('duration');

                                // check if intersects
                                if ((compareBegin <= begin && compareEnd > begin) ||
                                    (compareBegin < end && compareEnd >= end) ||
                                    (compareBegin <= begin && compareEnd >= end)
                                ) {
                                    available = false;
                                    return false;
                                }
                            }

                            return true;
                        });

                        if (available) {
                            view.ui.nodes.append(node.get('el'));
                            return false;
                        }

                        return true;
                    });

                    if (!available) {
                        self.addTimeline();
                        self.children.last().ui.nodes.append(node.get('el'));
                    }
                });

                // remove empty timelines
                while (this.children.last() && $('.app-node', this.children.last().ui.nodes).length == 0)
                    this.collection.pop();

                this.updateTimelineWidth();
            },
            addTimeline: function () {
                this.collection.add(new Backbone.Model({}));
            },
            removeSelected: function () {
                var self = this,
                    any = false,
                    nodes = [];

                // remove selected
                this.children.each(function (view) {
                    if (view.selected()) {
                        nodes = _.union(nodes, $('.app-node', view.el).toArray());
                        self.collection.remove(view.model);
                        any = true;
                    }
                });

                // remove last timeline if none selected
                if (!any && self.collection.length > 0) {
                    if (this.children.last()) {
                        nodes = $('.app-node', this.children.last().el).toArray();
                        if (nodes.length > 0)
                            self.collection.pop();
                    }
                }

                // destroy node models
                _.each(nodes, function (el) {
                    var node = self.model.get('nodes').findWhere({'el': el});

                    if (node)
                        node.destroy();
                });

                // uncheck select all checkbox
                this.ui.removeButton.prop('checked', false);
            },
            selectAll: function () {
                var checked = this.ui.selectAll.prop('checked');
                this.children.each(function (view) {
                    view.selected(checked);
                });
            },
            updateTimelineWidth: function () {
                if (this.children.length > 0) {
                    var self = this,
                        width = this.children.first().ui.nodes.width();

                    this.model.get('nodes').each(function (node) {
                        width = Math.max(width, (node.get('offset') + node.get('duration')) * self.config.pxPerSec);
                    });

                    if (width == 0)
                        width = '100%';

                    this.children.each(function (timeline) {
                        timeline.ui.nodes.css('width', width);
                    });
                }
            },
            setPerformanceName: function () {
                this.model.set('name', this.ui.performanceName.val());
            },
            saveClicked: function () {
                Views.trigger('performances:save');
            }
        });
    });
});
