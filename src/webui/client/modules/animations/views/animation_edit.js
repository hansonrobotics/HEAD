define(["application", "./templates/animation_edit.tpl"], function (App, template) {
    return Marionette.View.extend({
        template: template,
        ui: {
            name: '.app-name'
        },
        events: {
            'change @ui.name': 'updateAnimation'
        },
        modelEvents: {
            change: 'modelChanged'
        },
        onRender: function () {
            this.modelChanged();
        },
        modelChanged: function () {
            this.ui.name.val(this.model.get('name'));
        },
        updateAnimation: function () {
            this.model.set('name', this.ui.name.val());
        }
    });
});
