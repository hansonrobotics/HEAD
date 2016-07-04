define(['application', 'marionette', 'tpl!./templates/node.tpl', 'lib/api', 'underscore', './node_settings',
        'lib/regions/fade_in', '../entities/node', 'jquery-ui', 'lib/crosshair-slider', 'select2'],
    function (App, Marionette, template, api, _, NodeSettingsView, FadeInRegion, Node) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                container: '.app-node-settings',
                tabs: '.app-node-tabs li',
                nameButtons: '.app-node-tabs a',
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
                'click @ui.nameButtons': 'nameButtonClick'
            },
            initialize: function (options) {
                this.mergeOptions(options, ['node']);
            },
            onRender: function () {
                this.ui.nameButtons.draggable({
                    helper: function () {
                        return $('<span>').addClass('label app-node').attr('data-node-name',
                            $(this).data('node-name')).html($(this).html()).get(0);
                    },
                    appendTo: 'body',
                    revert: 'invalid',
                    delay: 100,
                    snap: '.app-timeline-nodes',
                    snapMode: 'inner',
                    zIndex: 1000
                });

                if (this.node)
                    this.showSettings(this.node);
                else
                    this.hideSettings();
            },
            hideSettings: function () {
                var self = this,
                    current = this.getRegion('settings').currentView;

                this.node = null;
                this.ui.tabs.removeClass('active');

                if (current) {
                    current.model.unbind('change:duration', this.updateNode, this);
                    current.$el.slideUp(null, function () {
                        current.destroy();
                        self.ui.container.addClass('app-disabled');
                    });
                } else
                    this.ui.container.addClass('app-disabled');
            },
            showSettings: function (node) {
                node.bind('change', this.updateNode, this);
                if (this.node) this.node.unbind('change', this.updateNode, this);
                this.node = node;
                this.ui.container.removeClass('app-disabled');
                this.ui.tabs.removeClass('active');
                this.ui.nameButtons.filter('[data-node-name="' + node.get('name') + '"]')
                    .closest('li').addClass('active');
                this.getRegion('settings').show(new NodeSettingsView({model: node}));
                this.updateIndicators();
            },
            nameButtonClick: function (e) {
                this.changeName($(e.target).data('node-name'));
            },
            changeName: function (name) {
                var currentView = this.getRegion('settings').currentView;
                if (currentView)
                    currentView.destroy();
                if (this.node) {
                    this.node.set('name', name);
                } else {
                    this.node = new Node({name: name, start_time: 0, duration: 1});
                    this.collection.add(this.node);
                }
                this.showSettings(this.node);
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
