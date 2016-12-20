/*
 * Backbone.js & Underscore.js Natural Sorting
 *
 * @author Kevin Jantzer <https://gist.github.com/kjantzer/7027717>
 * @since 2013-10-17
 *
 * NOTE: make sure to include the Natural Sort algorithm by Jim Palmer (https://github.com/overset/javascript-natural-sort)
 */

// add _.sortByNat() method
_.mixin({

    sortByNat: function(obj, value, context) {
        var iterator = _.isFunction(value) ? value : function(obj){ return obj[value]; };
        return _.pluck(_.map(obj, function(value, index, list) {
            return {
                value: value,
                index: index,
                criteria: iterator.call(context, value, index, list)
            };
        }).sort(function(left, right) {
            var a = left.criteria;
            var b = right.criteria;
            return naturalSort(a, b);
        }), 'value');
    }
});


// add _.sortByNat to Backbone.Collection
Backbone.Collection.prototype.sortByNat = function(value, context) {
    var iterator = _.isFunction(value) ? value : function(model) {
            return model.get(value);
        };
    return _.sortByNat(this.models, iterator, context);
};

// new Natural Sort method on Backbone.Collection
Backbone.Collection.prototype.sortNat = function(options) {
    if (!this.comparator) throw new Error('Cannot sortNat a set without a comparator');
    options || (options = {});

    if (_.isString(this.comparator) || this.comparator.length === 1) {
        this.models = this.sortByNat(this.comparator, this);
    } else {
        this.models.sortNat(_.bind(this.comparator, this));
    }

    if (!options.silent) this.trigger('sort', this, options);
    return this;
};

// save the oringal sorting method
Backbone.Collection.prototype._sort = Backbone.Collection.prototype.sort;

// override the default sort method to determine if "regular" or "natural" sorting should be used
Backbone.Collection.prototype.sort = function(){

    if( this.sortType && this.sortType === 'natural' )
        Backbone.Collection.prototype.sortNat.apply(this, arguments);
    else
        Backbone.Collection.prototype._sort.apply(this, arguments);
};