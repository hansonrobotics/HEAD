define(["application", "lib/api", './views/motors', './views/layout', './views/configuration', 'entities/motor'],
    function (App, api, MotorsView, LayoutView, ConfigurationView) {
        return {
            public_index: function () {
                // reset robot
                api.disableInteractionMode();
                api.blenderMode.disable();
                api.setDefaultMotorValues();

                // init collection and views
                var motorsCollection = new App.Entities.MotorCollection(),
                    motorsView = new MotorsView({collection: motorsCollection}),
                    layoutView = new LayoutView();

                motorsCollection.fetch();

                App.LayoutInstance.showNav();
                App.LayoutInstance.setTitle('Motors');
                App.LayoutInstance.getRegion('content').show(layoutView);

                layoutView.getRegion('motors').show(motorsView);
            },
            admin_index: function () {
                App.LayoutInstance.showAdminNav();
                App.LayoutInstance.setTitle('Motors');
                api.disableInteractionMode();
                api.blenderMode.disable();
                api.setDefaultMotorValues();
                var configurationView = new ConfigurationView();
                App.LayoutInstance.getRegion('content').show(configurationView);
            }
        };
    });
