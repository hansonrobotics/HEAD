define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Emotion = Backbone.Model.extend({
            initialize: function () {
                var self = this;

                if (! this.get('value'))
                    this.set('value', 0);

                this.on('change', function () {
                    if (self.previous('value') != self.get('value'))
                        api.setEmotion(self.get('name'), self.get('value'));
                });
            }
        });
        Entities.EmotionCollection = Backbone.Collection.extend({
            model: Entities.Emotion,
            fetch: function () {
                var self = this;

                api.getAvailableEmotionStates(function (emotions) {
                    _.each(emotions, function (name) {
                        self.add({name: name});
                    });
                });
            }
        });
    });
});
