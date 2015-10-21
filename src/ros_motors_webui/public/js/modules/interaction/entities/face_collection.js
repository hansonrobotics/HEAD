define(['backbone', 'lib/api'], function (Backbone, api) {
    var face = Backbone.Model.extend({
        getThumbnailUrl: function () {
            return '/public/faces/Face' + this.id + '.jpg';
        }
    });

    return Backbone.Collection.extend({
        model: face,
        fetch: function () {
            var self = this,
                callback = function (response) {
                    self.reset();
                    self.add(response.faces);
                };

            api.topics.available_soma_states.unsubscribe();
            api.topics.available_soma_states.removeAllListeners();
            api.topics.face_locations.subscribe(callback);
            api.topics.available_soma_states.unsubscribe();
            api.topics.available_soma_states.removeAllListeners();
        }
    });
});