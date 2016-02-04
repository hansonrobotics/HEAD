define(['jquery', 'marionette', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in'],
    function ($, Marionette, template, FadeInRegion) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                title: '#app-title',
                adminNav: '#app-admin-nav',
                nav: '#app-nav',
                navLinks: '#app-nav a, #app-admin-nav a',
                content: '#app-content',
                navbarContainer: '#navbar-container'
            },
            events: {
                'click @ui.navLinks': 'navLinkClicked'
            },
            regions: {
                content: {
                    el: "#app-content",
                    regionClass: FadeInRegion
                }
            },
            onRender: function () {
                this.setFluid(!!this.options.fluid);
                this.showNav();
            },
            setTitle: function (title) {
                this.ui.title.text(title);
            },
            showAdminNav: function () {
                if (!this.ui.adminNav.is(':visible')) {
                    this.ui.adminNav.show();
                    this.ui.nav.hide();
                }
            },
            showNav: function () {
                if (!this.ui.nav.is(':visible')) {
                    this.ui.nav.show();
                    this.ui.adminNav.hide();
                }
            },
            navLinkClicked: function () {
                $('.navbar-collapse.in').collapse('hide');
            },
            setFluid: function (enable) {
                if (enable) {
                    this.ui.navbarContainer.removeClass('container').addClass('container-fluid');
                    this.ui.content.removeClass('container').addClass('container-fluid');

                } else {
                    this.ui.navbarContainer.removeClass('container-fluid').addClass('container');
                    this.ui.content.removeClass('container-fluid').addClass('container');
                }
            }
        });
    });