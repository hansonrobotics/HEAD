define(["ros_ui", "apps/motor/show/list_controller"], function(UI, MotorController){
    UI.module("MotorApp", function(MotorApp, UI, Backbone, Marionette, $, _){
        MotorApp.on("start", function(){
            MotorController.list();
        });
    });

    return UI.MotorApp;
});
