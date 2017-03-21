define(['marionette', 'underscore', 'jquery', 'jquery-ui'], function(Marionette, _, $) {
    return Marionette.View.extend({
        tagName: 'a',
        template: false,
        attributes: {
            'class': 'app-performance-button btn btn-default',
            type: 'a'
        },
        modelEvents: {
            'change:name change:path': 'update'
        },
        initialize: function(options) {
            this.mergeOptions(options, ['readonly', 'nav', 'performancesView'])
        },
        onRender: function() {
            if (!this.readonly)
                this.$el.attr('data-cid', this.cid).draggable({
                    appendTo: 'body',
                    cancel: false,
                    revert: true,
                    helper: 'clone',
                    refreshPositions: true
                })
            this.update()
        },
        update: function() {
            this.$el.html(this.model.get('name'))
            this.$el.attr('data-path', this.model.get('path') || '')
                .attr('href', '#/performances/' + this.model.get('id'))

            if (!this.nav) {
                let self = this
                this.$el.attr('href', 'javascript:void(0)').click(function() {
                        self.performancesView.navigate(self.model.get('id'))
                    }
                )
            }
        }
    })
})
