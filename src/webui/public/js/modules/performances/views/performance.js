define(['marionette', 'underscore', 'jquery', 'jquery-ui'], function (Marionette, _, $) {
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
            'change:name change:path': 'update'
        },
        onRender: function () {
            this.$el.attr('data-cid', this.cid).draggable({
                appendTo: 'body',
                cancel: false,
                revert: true,
                helper: 'clone',
                refreshPositions: true
            });
            this.update();
        },
        update: function () {
            this.$el.html(this.model.get('name'));
            this.$el.attr('data-path', this.model.get('path') || '');
        }
    });
});
