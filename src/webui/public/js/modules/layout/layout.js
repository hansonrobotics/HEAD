define(['jquery', 'marionette', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in', 'bootbox',
    'lib/api'], function ($, Marionette, template, FadeInRegion, bootbox, api) {
    return Marionette.LayoutView.extend({
        template: template,
        ui: {
            title: '#app-title',
            navContainer: '#app-navbar',
            nav: '#app-nav',
            adminNav: '#app-admin-nav',
            navLinks: '#app-nav a, #app-admin-nav a',
            content: '#app-content',
            navbarContainer: '#navbar-container',
            reportButton: '.app-report-button',
            notifications: '#notifications'
        },
        events: {
            'click @ui.navLinks': 'navLinkClicked',
            'click @ui.reportButton': 'reportClicked'
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
        reportClicked: function (e) {
            e.preventDefault();
            bootbox.prompt({
                title: "Describe the problem you have found",
                inputType: 'textarea',
                callback: function (result) {
                    if (result !== null) api.bugReport(result);
                }
            });
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
        hideNav: function(){
            this.ui.nav.hide();
            this.ui.adminNav.hide();
            this.ui.notifications.hide();
            this.ui.reportButton.hide();
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
        },
        getContentHeight: function () {
            return window.innerHeight - this.ui.navContainer.outerHeight()
        }
    });
});