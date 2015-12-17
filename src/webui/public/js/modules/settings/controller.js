define(['application', './views/settings', './entities/robot_config', './entities/robot_config_schema'],
    function (App, SettingsView, RobotConfig, RobotConfigSchema) {
        return {
            settings: function () {
                var robotConfig = new RobotConfig(),
                    robotConfigSchema = new RobotConfigSchema();

                robotConfig.fetch();
                robotConfigSchema.fetch({
                    success: function (model) {
                        var settings = new SettingsView({model: robotConfig, schema: model.toJSON()});
                        App.LayoutInstance.getRegion('content').show(settings);
                    }
                });

                App.LayoutInstance.setTitle('Robot Settings');
                App.LayoutInstance.showAdminNav();
            }
        };
    });
