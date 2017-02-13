module.exports = Backbone.Collection.extend({
    model: require('./queue_item'),
    findItemByTime: function(time) {
        let offset = 0
        return this.find(function(item) {
            let match = time >= offset
            offset += item.get('performance').getDuration()
            return match && time < offset
        })
    }
})