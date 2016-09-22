define(['application', 'marionette', './templates/node.tpl', 'lib/api', 'underscore', './node_select',
        './node_settings', 'lib/regions/fade_in', '../entities/node', 'jquery', 'jquery-ui', 'lib/crosshair-slider', 'select2'],
    function (App, Marionette, template, api, _, NodeSelectView, NodeSettingsView, FadeInRegion, Node, $) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                container: '.app-node-settings',
                nodeTabBar: '.app-node-tabs',
                nodeTabs: '.app-node-tabs li:not([role="presentation"])',
                nodeScrollLeft: '.app-node-scroll.app-scroll-left a',
                nodeScrollRight: '.app-node-scroll.app-scroll-right a',
                nodeTypeButtons: '.app-node-tabs li:not([role="presentation"]) a',

                optionsTabBar: '.app-options-tabs',
                optionsTabs: '.app-options-tabs li:not([role="presentation"])',
                optionsScrollLeft: '.app-options-scroll.app-scroll-left a',
                optionsScrollRight: '.app-options-scroll.app-scroll-right a',

                hideButton: '.app-hide-settings-button',
                deleteButton: '.app-delete-node-button',
                frameCount: '.app-node-frames-indicator',
                durationIndicator: '.app-node-duration-indicator'
            },
            regions: {
                select: {
                    el: '.app-node-select-content',
                    regionClass: FadeInRegion
                },
                settings: {
                    el: '.app-settings-content',
                    regionClass: FadeInRegion
                }
            },
            events: {
                'click @ui.hideButton': 'hideSettings',
                'click @ui.deleteButton': 'deleteNode',
                'click @ui.nodeTypeButtons': 'typeButtonClick'
            },
            initialize: function (options) {
                this.mergeOptions(options, ['node']);
            },
            onAttach: function () {
                var self = this;
                this.ui.nodeTypeButtons.draggable({
                    helper: function () {
                        var attributes;
                        if (self.node && self.collection && self.node.get('name') == $(this).data('node-name')) {
                            // Copy current view attributes if node is same
                            attributes = self.node.toJSON();
                            delete attributes['id'];

                        } else {
                            attributes = {
                                name: $(this).data('node-name'),
                                duration: 1
                            }
                        }
                        var node = Node.create(attributes);
                        node.setDefaultValues();
                        return $('<span>').attr('data-node-name', node.get('name'))
                            .attr('data-node-id', node.get('id'))
                            .addClass('label app-node').html(node.getTitle());
                    },
                    appendTo: 'body',
                    revert: 'invalid',
                    delay: 100,
                    snap: '.app-timeline-nodes',
                    snapMode: 'inner',
                    zIndex: 1000,
                    cursor: 'move',
                    cursorAt: {top: 0, left: 0}
                });

                if (this.node)
                    this.showSettings(this.node);
                else
                    this.hideSettings();

                this.initTabScrolling(this.ui.nodeTabBar, this.ui.nodeTabs, this.ui.nodeScrollLeft, this.ui.nodeScrollRight);
                this.initTabScrolling(this.ui.optionsTabBar, this.ui.optionsTabs, this.ui.optionsScrollLeft, this.ui.optionsScrollRight);

                this.initResponsive();
            },
            initResponsive: function () {
                var self = this,
                    resize = function () {
                        if (self.isDestroyed)
                            $(window).unbind('resize', resize);
                        else if (self.ui.nodeTabBar.offset().left < self.ui.optionsTabBar.offset().left)
                            self.ui.container.removeClass('app-mobile');
                        else
                            self.ui.container.addClass('app-mobile');
                    };

                $(window).bind('resize', resize);
            },
            initTabScrolling: function (tabBar, tabs, scrollLeft, scrollRight) {
                var self = this,
                    resize = function () {
                        if (self.isDestroyed) {
                            $(window).off('resize', resize);
                            return;
                        }

                        var width = self.getWidthSum(tabs);

                        if (width > $(tabBar).width()) {
                            $(tabBar).addClass('app-overflow');
                            $(scrollLeft).fadeIn();
                            $(scrollRight).fadeIn();
                        } else {
                            $(tabBar).removeClass('app-overflow');
                            $('li:not([role="presentation"])', tabBar).css('right', 0);
                            $(scrollLeft).hide();
                            $(scrollRight).hide();
                        }
                    },
                    eventData = {
                        tabBar: tabBar,
                        tabs: tabs,
                        scrollLeft: scrollLeft,
                        scrollRight: scrollRight
                    };

                $(scrollLeft).click(eventData, function (e) {
                    $(this).blur();
                    $(e.data.tabs).stop().animate({right: Math.max(parseInt(e.data.tabs.css('right')) - 200, 0)});
                });

                $(scrollRight).click(eventData, function (e) {
                    $(this).blur();
                    var max = self.getWidthSum(e.data.tabs) - e.data.tabBar.width();
                    $(e.data.tabs).stop().animate({right: Math.min(parseInt(e.data.tabs.css('right')) + 200, max)});
                });

                $(window).resize(resize);
                resize();
            },
            /**
             * Calculate precise sum of node tab widths
             * @param tabs
             * @returns {number}
             */
            getWidthSum: function (tabs) {
                var width = 0;
                $(tabs).each(function () {
                    var rect = $(this).get(0).getBoundingClientRect();
                    if (rect.width) {
                        // `width` is available for IE9+
                        width += rect.width;
                    } else {
                        // Calculate width for IE8 and below
                        width += rect.right - rect.left;
                    }
                });
                return width;
            },
            hideSettings: function () {
                var self = this,
                    selectView = this.getRegion('select').currentView,
                    settingsView = this.getRegion('settings').currentView;

                this.stopListening(this.node, 'change');
                this.node = null;
                this.ui.nodeTabs.removeClass('active');
                $('.app-node.active').removeClass('active');
                if (settingsView) {
                    $(selectView.$el).add(settingsView.$el).slideUp(null, function () {
                        self.ui.container.addClass('app-disabled');
                    });
                } else {
                    this.ui.container.addClass('app-disabled');
                }
            },
            showSettings: function (node) {
                if (node === this.node) return;
                this.node = node;
                this.listenTo(node, 'change', this.updateNode);
                this.updateNode(node);
                this.ui.container.removeClass('app-disabled');
                this.ui.nodeTabs.removeClass('active');
                this.ui.nodeTypeButtons.filter('[data-node-name="' + node.get('name') + '"]').closest('li').addClass('active');
                this.getRegion('select').show(new NodeSelectView({model: node, collection: this.collection}));
                this.getRegion('settings').show(new NodeSettingsView({model: node, collection: this.collection}));
                this.updateIndicators();
                $(window).resize();
            },
            typeButtonClick: function (e) {
                this.createNode($(e.target).data('node-name'));
            },
            createNode: function (name) {
                var node = Node.create({name: name, start_time: 0, duration: 1});
                this.showSettings(node);
            },
            updateNode: function (node) {
                if (this.node != node)
                    this.stopListening(node);
                else {
                    if (node.changed['duration']) this.updateIndicators();
                    if (this.collection.contains(node)) this.ui.deleteButton.fadeIn();
                    else this.ui.deleteButton.hide();
                }
            },
            updateIndicators: function () {
                var fps = App.getOption('fps'),
                    step = 1 / fps,
                    duration = Number(parseInt(this.node.get('duration') / step) * step).toFixed(2);

                this.ui.durationIndicator.html(duration + 's');
                this.ui.frameCount.html(parseInt(duration * fps));
            },
            deleteNode: function () {
                this.collection.remove(this.node);
                if (this.node) this.node.destroy();
                this.hideSettings();
            }
        });
    });
