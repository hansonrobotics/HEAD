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
        initialize: function(options) {
            this.mergeOptions(options, ['layoutView', 'readonly'])
        },
        childViewOptions: function() {
            return this.options
        }
    })
})
