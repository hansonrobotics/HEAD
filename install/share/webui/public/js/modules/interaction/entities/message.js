define(['backbone', 'lib/api'], function (Backbone) {
    return Backbone.Model.extend({
        initialize: function () {
            this.set('time_created', new Date().getTime());
        }
    });
});
