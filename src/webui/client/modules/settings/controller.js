define(['application', './views/layout', './views/settings', './entities/robot_config', './entities/robot_config_schema',
        './entities/node_config', './entities/node_config_schema', './css/settings'],
    function(App, LayoutView, SettingsView, RobotConfig, RobotConfigSchema, NodeConfig, NodeConfigSchema) {
        let self = {
            showLayout: function() {
                if (!this.layout || this.layout !== App.LayoutInstance.getRegion('content').currentView) {
                    this.layout = new LayoutView()
                    App.LayoutInstance.showAdminNav()
                    App.LayoutInstance.setTitle('Settings')
                    App.LayoutInstance.setFluid(true)
                    App.LayoutInstance.getRegion('content').show(this.layout)
                }
            },
            robot: function() {
                self.showLayout()

                // let robotConfig = new RobotConfig(),
                //     robotConfigSchema = new RobotConfigSchema()
                //
                // robotConfig.fetch()
                // robotConfigSchema.fetch({
                //     success: function(model) {
                //         let settings = new SettingsView({model: robotConfig, schemaModel: model})
                //         self.layout.getRegion('content').show(settings)
                //     }
                // })
            },
            node: function(node) {
                self.showLayout()

                node = decodeURIComponent(node)
                let nodeConfig = new NodeConfig(node),
                    nodeConfigSchema = new NodeConfigSchema(node)

                nodeConfig.fetch()
                nodeConfigSchema.fetch({
                    success: function(model) {
                        let settings = new SettingsView({model: nodeConfig, schemaModel: model, refresh: true})
                        self.layout.getRegion('content').show(settings)
                    }
                })
            }
        }
        return self
    })
