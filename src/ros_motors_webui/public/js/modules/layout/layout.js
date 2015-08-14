define(['jquery', 'marionette', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in'],
    function ($, Marionette, template, FadeInRegion) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                title: '#app-title',
                adminNav: '#app-admin-nav',
                nav: '#app-nav',
                navLinks: '#app-nav a, #app-admin-nav a'
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
            }
        });
    });