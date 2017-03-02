define(['marionette', './templates/layout.tpl', 'lib/regions/fade_in', 'jquery', 'lib/api', 'underscore',
        'multilevelpushmenu', 'multilevelpushmenu-css'],
    function(Marionette, template, FadeInRegion, $, api, _) {
        return Marionette.View.extend({
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
                navLinks: '.app-settings-navigation a'
            },
            events: {
                'click @ui.navLinks': 'collapseNav'
            },
            onDomRefresh: function() {
                let self = this,
                    updateNavHeight = function() {
                        if (self.isDestroyed())
                            $(window).off("resize", updateNavHeight)
                        else {
                            self.ui.navigation.multilevelpushmenu('option', 'menuHeight', $(document).height())
                            self.ui.navigation.multilevelpushmenu('redraw')
                        }
                    }

                this.ui.navigation.multilevelpushmenu({
                    container: this.ui.navigation,
                    menuWidth: '250px',
                    menuHeight: $(document).height(),
                    collapsed: true,
                    preventItemClick: false
                })

                $(window).on("resize", updateNavHeight)
                this.nodeListInterval = setInterval(_.bind(this.updateNodeList, this), 10000)
                this.updateNodeList()
            },
            collapseNav: function(e) {
                $(this.ui.navigation).multilevelpushmenu('collapse')
            },
            updateNodeList: function() {
                let self = this
                api.services.get_configurable_nodes.callService({}, function(response) {
                    let container = $('<div>')
                    $.each(response.nodes, function() {
                        let link = $('<a>').prop({
                            href: '#/admin/settings/node/' + encodeURIComponent(this)
                        }).html(this)
                        container.append($('<li>').append(link))
                    })
                    self.ui.nodeList.html(container.html())
                }, function(error) {
                    console.log(error)
                })
            },
            onDestroy: function() {
                $(window).off("resize", this.updateNavHeight, this)
                clearInterval(this.nodeListInterval)
            }
        })
    })
