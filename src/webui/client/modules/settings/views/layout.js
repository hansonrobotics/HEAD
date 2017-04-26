define(['marionette', './templates/layout.tpl', 'lib/regions/fade_in', 'jquery', 'lib/api', 'underscore',
        'multilevelpushmenu', 'multilevelpushmenu-css', 'scrollbar'],
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
                settingsContainer: '.app-settings-container',
                navigation: '.app-settings-navigation',
                nav: '.app-settings-navigation > nav',
                nodeList: '.app-nav-node-list',
                navLinks: '.app-settings-navigation a'
            },
            events: {
                'click @ui.navLinks': 'navLinkClick'
            },
            config: {
                mobileWidth: 800
            },
            initialize: function() {
                // load first node
                this.init = true
            },
            onAttach: function() {
                this.mobile = $(window).width() < this.config.mobileWidth;
                this.ui.navigation.multilevelpushmenu({
                    container: this.ui.navigation,
                    menuWidth: '250px',
                    menuHeight: $(window).height(),
                    collapsed: this.mobile,
                    swipe: false
                })

                _.bindAll(this, ['resizeCallback'])
                this.resizeCallback()

                $(this.ui.nav).perfectScrollbar()
                $(window).on("resize", this.resizeCallback)

                this.nodeListInterval = setInterval(_.bind(this.updateNodeList, this), 10000)
                this.updateNodeList()
            },
            resizeCallback: function() {
                this.ui.navigation.multilevelpushmenu('option', 'menuHeight', $(window).height())
                this.ui.navigation.multilevelpushmenu('redraw')
                $(this.ui.nav).perfectScrollbar('update')

                this.mobile = $(window).width() < this.config.mobileWidth;

                if (this.mobile) {
                    this.ui.navigation.find('.cursorPointer').fadeIn()
                    this.ui.settingsContainer.addClass('mobile')
                } else {
                    this.ui.navigation.find('.cursorPointer').hide()
                    this.ui.settingsContainer.removeClass('mobile')
                    this.expandNav()
                }
            },
            navLinkClick: function(e) {
                if (this.mobile)
                    this.collapseNav()
                $(e.target).blur()
            },
            expandNav: function(e) {
                $(this.ui.navigation).multilevelpushmenu('expand')
            },
            collapseNav: function(e) {
                $(this.ui.navigation).multilevelpushmenu('collapse')
            },
            updateNodeList: function() {
                let self = this
                api.services.get_configurable_nodes.callService({}, function(response) {
                    let container = $('<div>')

                    $.each(response.nodes, function() {
                        let url = '/admin/settings/node/' + encodeURIComponent(this)

                        // go to first node settings on initialization
                        if (self.init) {
                            self.init = false
                            Backbone.history.navigate(url, {trigger: true})
                        }
                        let link = $('<a>').prop({
                            href: '#' + url
                        }).html(this)
                        container.append($('<li>').append(link))
                    })
                    self.ui.nodeList.html(container.html())
                    $(self.ui.nav).perfectScrollbar('update')
                }, function(error) {
                    console.log(error)
                })
            },
            onDestroy: function() {
                clearInterval(this.nodeListInterval)
                $(window).off("resize", this.resizeCallback)
            }
        })
    })
