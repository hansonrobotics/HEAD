define(['marionette', 'backbone', './timelines', 'underscore', 'lib/regions/fade_in',
    '../entities/performance', 'lib/behaviors/sortable'], function(Marionette, Backbone, TimelinesView, _, FadeInRegion, Performance) {
    return Marionette.CollectionView.extend({
        childView: require('./item'),
        emptyView: require('./empty_item'),
        tagName: 'ul',
        className: 'app-performance-queue list-group',
        ui: {},
        behaviors: [{
            behaviorClass: Marionette.SortableBehavior,
            axis: "y",
            handle: ".app-drag-handle",
            placeholder: "ui-state-highlight",
        }],
        events: {
            'sortupdate': 'onSortUpdate'
        },
        initialize: function(options) {
            this.mergeOptions(options, ['layoutView', 'readonly', 'playEnabled'])
        },
        childViewOptions: function() {
            return _.extend(this.options, {playEnabled: this.playEnabled})
        },
        enablePlay: function() {
            this.playEnabled = true
            if (this.collection.length)
                this.children.each(function(item, a2) {
                    item.enablePlay()
                })
        },
        disablePlay: function() {
            this.playEnabled = false
            if (this.collection.length)
                this.children.each(function(item) {
                    item.disablePlay()
                })
        },
        onSortUpdate: function() {
            this.trigger('reordered')
        }
    })
})
