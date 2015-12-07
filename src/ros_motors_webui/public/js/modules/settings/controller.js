define(['application', './views/settings', './entities/robot_config'], function (App, SettingsView, RobotConfig) {
    return {
        settings: function () {
            var robotConfig = new RobotConfig(),
                settings = new SettingsView({model: robotConfig});

            robotConfig.fetch();

            App.LayoutInstance.setTitle('Robot Settings');
            App.LayoutInstance.getRegion('content').show(settings);
            App.LayoutInstance.showAdminNav();
        }
    };
});
