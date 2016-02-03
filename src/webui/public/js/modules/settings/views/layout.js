define(['marionette', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in', 'jquery', 'lib/api', 'push_menu'],
    function (Marionette, template, FadeInRegion, $, api) {
        var self = this;
        return Marionette.LayoutView.extend({
            template: template,
            regions: {
                content: {
                    el: '.app-settings-container',
                    regionClass: FadeInRegion
                }
            },
            ui: {
                navigation: '.app-settings-navigation',
                nodeList: '.app-nav-node-list',
                nodeLinks: '.app-nav-node-list a',
                robotSettingsLink: '.app-robot-settings'
            },
            events: {
                'click @ui.nodeLinks': 'nodeClicked',
                'click @ui.robotSettingsLink': 'robotClicked'
            },
            nodeClicked: function (e) {
                self.trigger('node_settings', $(e.target).html());
                self.collapseNav();
            },
            robotClicked: function () {
                self.trigger('robot_settings');
            },
            initialize: function () {
                self = this;
            },
            onShow: function () {
                this.ui.navigation.multilevelpushmenu({
                    menuWidth: '250px',
                    menuHeight: $(document).height(),
                    collapsed: true,
                    preventItemClick: false,
                    onItemClick: function () {
                        self.collapseNav();
                    }
                });

                $(window).on("resize", this.updateNavHeight);
                this.nodeListInterval = setInterval(this.updateNodeList, 10000);
                this.updateNodeList();
            },
            collapseNav: function () {
                $(this.ui.navigation).multilevelpushmenu('collapse');
            },
            updateNodeList: function () {
                api.services.get_configurable_nodes.callService({}, function (response) {
                    var container = $('<div>');
                    $.each(response.nodes, function () {
                        var link = $('<a>').prop({
                            href: '#/admin/settings'
                        }).html(this);
                        container.append($('<li>').append(link));
                    });
                    self.ui.nodeList.html(container.html());
                }, function (error) {
                    console.log(error);
                });
            },
            updateNavHeight: function () {
                self.ui.navigation.multilevelpushmenu('option', 'menuHeight', $(document).height());
                self.ui.navigation.multilevelpushmenu('redraw');
            },
            onDestroy: function () {
                $(window).off("resize", this.updateNavHeight);
                clearInterval(this.nodeListInterval);
            }
        });
    });
