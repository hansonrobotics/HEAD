define(['application', 'lib/api'], function (App, api) {
    return Backbone.Model.extend({
        idAttribute: 'node',
    });
});

