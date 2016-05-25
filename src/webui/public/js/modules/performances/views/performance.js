define(['marionette'], function (Marionette) {
    return Marionette.ItemView.extend({
        tagName: 'button',
        template: false,
        attributes: {
            'class': 'app-performance-button btn btn-default',
            type: 'button'
        },
        triggers: {
            click: "click" // fires a 'click' event on view instance
        },
        modelEvents: {
            change: 'render'
        },
        onRender: function () {
            this.$el.html(this.model.get('name'));
            this.$el.attr('data-path', this.model.get('path').split('/').slice(0, -1).join('/'));
        }
    });
});
