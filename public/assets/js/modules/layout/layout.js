define(['marionette', 'tpl!./templates/layout.tpl'], function (Marionette, template) {
    return Marionette.LayoutView.extend({
        template: template,
        ui: {
            title: '#app-title',
            adminNav: '#app-admin-nav',
            nav: '#app-nav'
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
            this.ui.adminNav.show();
            this.ui.nav.hide();
        },
        showNav: function () {
            this.ui.nav.show();
            this.ui.adminNav.hide();
        }
    });
});