define(["../../ros_ui", "apps/motor/list/list_controller"], function(UI, MotorController){
    UI.module("MotorApp", function(MotorApp, UI, Backbone, Marionette, $, _){
        MotorApp.on("start", function(){
            MotorController.list();
        });
    });

    return UI.MotorApp;
});
