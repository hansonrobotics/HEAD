define(['application', './views/layout', './views/settings', './entities/robot_config', './entities/robot_config_schema',
        './entities/node_config', './entities/node_config_schema'],
    function (App, LayoutView, SettingsView, RobotConfig, RobotConfigSchema, NodeConfig, NodeConfigSchema) {
        var self = {
            showLayout: function () {
                if (!this.layout || this.layout != App.LayoutInstance.getRegion('content').currentView) {
                    this.layout = new LayoutView();
                    this.layout.on('node_selected', this.node);
                    App.LayoutInstance.showAdminNav();
                    App.LayoutInstance.setTitle('Settings');
                    App.LayoutInstance.getRegion('content').show(this.layout);
                }
            },
            robot: function () {
                this.showLayout();

                var robotConfig = new RobotConfig(),
                    robotConfigSchema = new RobotConfigSchema();

                robotConfig.fetch();
                robotConfigSchema.fetch({
                    success: function (model) {
                        var settings = new SettingsView({model: robotConfig, schema: model.toJSON()});
                        self.layout.getRegion('content').show(settings);
                    }
                });
            },
            node: function (node) {
                var nodeConfig = new NodeConfig(node),
                    nodeConfigSchema = new NodeConfigSchema(node);

                nodeConfig.fetch();
                nodeConfigSchema.fetch({
                    success: function (model) {
                        var settings = new SettingsView({model: nodeConfig, schema: model.toJSON()});
                        self.layout.getRegion('content').show(settings);
                    }
                });
            }
        };
        return self;
    });
