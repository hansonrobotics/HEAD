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

                motorsCollection.fetchFromParam();

                App.LayoutInstance.showNav();
                App.LayoutInstance.getRegion('content').show(layoutView);
                layoutView.getRegion('motors').show(motorsView);
            },
            admin_index: function () {
                App.LayoutInstance.showAdminNav();
                var configurationView = new ConfigurationView();
                App.LayoutInstance.getRegion('content').show(configurationView);
            }
        };
    });
