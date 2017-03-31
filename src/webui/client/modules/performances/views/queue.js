define(['marionette', 'backbone', './timelines', 'underscore', 'lib/regions/fade_in',
        '../entities/performance', 'lib/behaviors/sortable', 'scrollbar', 'scrollbar-css'],
    function(Marionette, Backbone, TimelinesView, _, FadeInRegion, Performance) {
        return Marionette.CollectionView.extend({
            childView: require('./item'),
            emptyView: require('./empty_item'),
            tagName: 'ul',
            className: 'app-performance-queue list-group',
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
                this.mergeOptions(options, ['layoutView', 'readonly', 'playEnabled', 'height'])
            },
            onAttach: function() {
                if (this.height) {
                    let self = this
                    this.$el.css('maxHeight', this.height).perfectScrollbar()
                    this.listenTo(this.collection, 'update reset', function() {
                        self.$el.perfectScrollbar('update')
                    })
                }
            },
            childViewOptions: function() {
                return _.extend(this.options, {queueView: this, readonly: this.readonly})
            },
            enablePlay: function() {
                this.playEnabled = true
                if (this.collection.length)
                    this.children.each(function(item) {
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
