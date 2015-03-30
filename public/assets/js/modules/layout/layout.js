define(['jquery', 'marionette', 'tpl!./templates/layout.tpl'], function ($, Marionette, template) {
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
            content: "#app-content"
        },
        onRender: function () {
            this.showNav();
        },
        setTitle: function (title) {
            this.ui.title.text(title);
        },
        showAdminNav: function () {
            if (! this.ui.adminNav.is(':visible')) {
                this.ui.adminNav.show();
                this.ui.nav.hide();
            }
        },
        showNav: function () {
            if (! this.ui.nav.is(':visible')) {
                this.ui.nav.show();
                this.ui.adminNav.hide();
            }
        },
        navLinkClicked: function () {
            $('.navbar-collapse.in').collapse('hide');
        }
    });
});