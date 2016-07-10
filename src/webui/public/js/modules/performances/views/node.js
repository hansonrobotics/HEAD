define(['application', 'marionette', 'tpl!./templates/node.tpl', 'lib/api', 'underscore', './node_settings',
        'lib/regions/fade_in', '../entities/node', 'jquery', 'jquery-ui', 'lib/crosshair-slider', 'select2'],
    function (App, Marionette, template, api, _, NodeSettingsView, FadeInRegion, Node, $) {
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
                if (this.node)
                    this.showSettings(this.node);
                else
                    this.hideSettings();

                this.initTabScrolling(this.ui.nodeTabBar, this.ui.nodeTabs, this.ui.nodeScrollLeft, this.ui.nodeScrollRight);
                this.initTabScrolling(this.ui.optionsTabBar, this.ui.optionsTabs, this.ui.optionsScrollLeft, this.ui.optionsScrollRight);
            },
            /**
             * Calculate precise sum of widths
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
                    };

                var eventData = {
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
            hideSettings: function () {
                var self = this,
                    current = this.getRegion('settings').currentView;

                this.node = null;
                this.ui.nodeTabs.removeClass('active');
                $('.app-node.active').removeClass('active');

                if (current) {
                    current.model.unbind('change:duration', this.updateNode, this);
                    current.$el.slideUp(null, function () {
                        current.destroy();
                        self.ui.container.addClass('app-disabled');
                        $(window).resize();
                    });
                } else {
                    this.ui.container.addClass('app-disabled');
                    $(window).resize();
                }
            },
            showSettings: function (node) {
                node.bind('change', this.updateNode, this);
                if (this.node) this.node.unbind('change', this.updateNode, this);
                this.node = node;
                this.ui.container.removeClass('app-disabled');
                this.ui.nodeTabs.removeClass('active');
                this.ui.nodeTypeButtons.filter('[data-node-name="' + node.get('name') + '"]')
                    .closest('li').addClass('active');
                this.getRegion('settings').show(new NodeSettingsView({model: node, collection: this.collection}));
                this.updateIndicators();
                $(window).resize();
            },
            typeButtonClick: function (e) {
                this.createNode($(e.target).data('node-name'));
            },
            createNode: function (name) {
                var currentView = this.getRegion('settings').currentView;
                if (currentView)
                    currentView.destroy();

                var node = new Node({name: name, start_time: 0, duration: 1});
                this.showSettings(node);
            },
            updateNode: function (node) {
                if (this.node != node)
                    node.unbind('change', this.updateNode, this);
                else {
                    if (node.changed['duration']) this.updateIndicators();
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
                if (this.node) this.node.destroy();
                this.hideSettings();
            }
        });
    });
