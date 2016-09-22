define(['backbone', 'lib/api'], function (Backbone, api) {
    return Backbone.Model.extend({
        initialize: function () {
            var self = this;

            if (!this.get('value'))
                this.set('value', 0);

            this.on('change', function () {
                if (self.previous('value') != self.get('value'))
                    api.setEmotion(self.get('name'), self.get('value'));
            });
        }
    });
});
