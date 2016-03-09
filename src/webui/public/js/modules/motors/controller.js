define(["application", "lib/api", './views/motors', './views/layout', './views/configuration', 'entities/motor'],
    function (App, api, MotorsView, LayoutView, ConfigurationView) {
        return {
            public_index: function () {
                App.LayoutInstance.setTitle('Motors');
                api.disableInteractionMode();
                api.blenderMode.disable();

                // init collection and views
                var motorsCollection = new App.Entities.MotorCollection(),
                    motorsView = new MotorsView({collection: motorsCollection}),
                    layoutView = new LayoutView();

                motorsCollection.fetchFromParam(function () {
                    motorsCollection.setDefaultValues();
                });

                App.LayoutInstance.showNav();
                App.LayoutInstance.getRegion('content').show(layoutView);
                layoutView.getRegion('motors').show(motorsView);
            },
            admin_index: function () {
                api.disableInteractionMode();
                api.blenderMode.disable();
                api.setExpression("Neutral", 0);
                api.pointHead();

                App.LayoutInstance.showAdminNav();
                App.LayoutInstance.setTitle('Motor configuration');
                var configurationView = new ConfigurationView();
                App.LayoutInstance.getRegion('content').show(configurationView);
            }
        };
    });
