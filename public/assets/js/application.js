define(["vendor/backbone.marionette", 'lib/api'], function (Marionette, api) {
    var Application = new Marionette.Application();
    Application.getAdminEnabled = function (callback) {
        api.getAdminEnabled(callback);
    };
    return Application;
});
