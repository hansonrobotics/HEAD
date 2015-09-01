define(['application'], function (App) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Performance = Marionette.ItemView.extend({
            tagName: 'button',
            template: false,
            attributes: {
                'class': 'app-performance-button btn btn-default',
                type: 'button'
            },
            events: {
                'click': 'click'
            },
            modelEvents: {
                'change': 'render'
            },
            onRender: function () {
                this.$el.html(this.model.get('name'));
            },
            click: function (e) {
                $('.app-performance-button').removeClass('active');
                $(e.target).addClass('active');
                Views.trigger('performance:click', this.model);
            }
        });
    });
});
