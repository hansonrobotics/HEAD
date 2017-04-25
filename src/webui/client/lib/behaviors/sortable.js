// Marionette.SortableCollectionView
//
// Copyright (c) 2014 orangain
// Distributed under MIT License
// https://github.com/orangain/marionette-sortable

(function(root, factory) {
    if (typeof define === 'function' && define.amd) {
        // AMD. Register as an anonymous module.
        define(['marionette', 'underscore'], factory)
    } else if (typeof exports !== 'undefined') {
        // Node/CommonJS
        let Marionette = require('marionette')
        let _ = require('underscore')
        factory(Marionette, _)
    } else {
        // Browser globals
        factory(root.Marionette, root._)
    }

}(this, function(Marionette, _) {

    Marionette.SortableBehavior = Marionette.Behavior.extend({
        events: {
            'sortupdate': 'onSortUpdate'
        },
        onSortUpdate: function(e, ui) {
            let $childElement = ui.item
            let newIndex = $childElement.parent().children().index($childElement)
            let collection = this.view.collection
            let model = collection.get($childElement.attr('data-model-cid'))
            // do not use silent to notify other obversers.
            collection.remove(model)
            collection.add(model, {at: newIndex})
        },
        onRender: function() {
            let options = _.clone(this.options)
            delete options.behaviorClass
            delete options.html5sortable

            this.getChildViewContainer().sortable(options) // options are passed to the sortable
        },
        onAddChild: function(view, childView) {
            childView.$el.attr('data-model-cid', childView.model.cid)
            if (this.options.html5sortable) {
                this.getChildViewContainer().sortable('reload')
            }
        },
        getChildViewContainer: function() {
            if (typeof this.view.getChildViewContainer === 'function') {
                // CompositeView
                return this.view.getChildViewContainer(this.view)
            } else if (typeof this.view.getItemViewContainer === 'function') {
                // CompositeView for Marionttte 1.x
                return this.view.getItemViewContainer(this.view)
            } else {
                // CollectionView
                return this.$el
            }
        }
    })
    Marionette.SortableCollectionView = Marionette.CollectionView.extend({
        constructor: function(options) {
            _.extend(this, _.pick(options, ['sortableOptions']))

            // Add Sortable behavior using sortableOptions
            this.behaviors = this.behaviors || {}
            this.behaviors._Sortable = _.extend({
                behaviorClass: Marionette.SortableBehavior
            }, this.sortableOptions || {})

            Marionette.CollectionView.apply(this, arguments)
        }
    })
}))