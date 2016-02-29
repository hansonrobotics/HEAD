define(['backbone', './message'], function (Backbone, Message) {
    return Backbone.Collection.extend({
        model: Message,
        comparator: function (item) {
            // two second priority for user messages
            return [item.get('time_created') / 3000, item.get('author') == 'Me' ? 1 : 2];
        }
    });
});
