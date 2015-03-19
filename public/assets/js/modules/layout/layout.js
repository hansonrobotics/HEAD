define(['marionette', 'tpl!./templates/layout.tpl'], function (Marionette, template) {
    return Marionette.LayoutView.extend({
        template: template,
        ui: {
            title: '#app-title'
        },
        regions: {
            content: "#app-content"
        },
        setTitle: function (title) {
            this.ui.title.text(title);
        }
    });
});