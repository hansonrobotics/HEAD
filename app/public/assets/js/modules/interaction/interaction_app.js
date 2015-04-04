define(['application', './controller'], function (App, controller) {
    App.module('Interaction', function (Interaction, app, Backbone, Marionette, $, _) {
        Interaction.Router = Marionette.AppRouter.extend({
            'appRoutes': {
                'interactions': 'index'
            }
        });

        Interaction.on('start', function () {
            new Interaction.Router({controller: controller});
        });
    });
});
