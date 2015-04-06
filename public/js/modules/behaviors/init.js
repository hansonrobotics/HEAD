define(['marionette', 'application'], function (Marionette, App) {
    Marionette.Behaviors.behaviorsLookup = function() {
        return App.Behaviors;
    }
});