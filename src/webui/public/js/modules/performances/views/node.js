define(['application', 'marionette', 'tpl!./templates/node.tpl', 'lib/api', 'underscore', './node_settings',
    'lib/regions/fade_in', 'jquery-ui', 'lib/crosshair-slider', 'select2'],
    function (App, Marionette, template, api, _, NodeSettingsView, FadeInRegion) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                container: '.app-node-settings',
                nodeTabs: '.app-node-tabs a'
            },
            regions: {
                settings: {
                    el: '.app-settings-content',
                    regionClass: FadeInRegion
                }
            },
            onRender: function () {
                this.ui.nodeTabs.draggable({
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
                this.showSettings();
            },
            hideSettings: function () {
                this.node = null;
                this.ui.container.addClass('app-disabled');
                this.nodeSettingsView.destroy();
            },
            showSettings: function (node) {
                if (this.node != node) {
                    this.node = node;
                    this.ui.container.removeClass('app-disabled');
                    this.nodeSettingsView = new NodeSettingsView({model: node});
                    this.getRegion('settings').show(this.nodeSettingsView);
                }
            }
        });
    });
