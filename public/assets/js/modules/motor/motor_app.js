define(["application", "./controller"], function (App, controller) {
    App.module("Motors", function (MotorApp, app, Backbone, Marionette, $, _) {
        controller.show();
    });

    return App.MotorApp;
});
