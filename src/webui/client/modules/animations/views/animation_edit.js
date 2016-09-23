define(["application", "./templates/animation_edit.tpl"], function (App, template) {
    App.module("Animations.Views", function (View, App, Backbone, Marionette, $, _) {
        View.AnimationEdit = Marionette.ItemView.extend({
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

    return App.Animations.Views.AnimationEdit;
});
