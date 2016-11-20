define(['application', 'lib/api'], function (App, api) {
    return Backbone.Model.extend({
        idAttribute: 'node',
        initialize: function (options) {
            console.log(options);
            console.log(this);
        }
    });
});

