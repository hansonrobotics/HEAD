define(['application'], function (App) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Performance = Marionette.ItemView.extend({
            tagName: 'button',
            template: false,
            attributes: {
                'class': 'btn btn-default',
                type: 'button'
            },
            events: {
                'click': 'click'
            },
            onRender: function () {
                this.$el.html(this.model.get('name'));
            },
            click: function () {
                Views.trigger('performance:click', this.model);
            }
        });
    });
});
