define(['application', './views/layout', 'modules/motors/views/motors', './views/messages', './views/logs',
        './views/speed', './views/system', './views/processes', 'entities/motor', './entities/log', './entities/ros_node'],
    function (App, LayoutView, MotorsView, MessagesView, LogsView, SpeedView, SystemView, processesVIew) {
        return {
            init: function (name) {
                if (!this.layoutView) {
                    var self = this;

                    this.layoutView = new LayoutView();
                    this.layoutView.on('destroy', function () {
                        delete self.layoutView;
                    });

                    App.LayoutInstance.setTitle('Monitoring');
                    App.LayoutInstance.getRegion('content').show(this.layoutView);
                    App.LayoutInstance.showAdminNav();
                }

                this.layoutView.setActive(name);
            },
            motors: function () {
                this.init('motors');
                var motorsCollection = new App.Entities.MotorCollection(),
                    motorsView = new MotorsView({
                    collection: motorsCollection,
                    monitoring: true
                });

                motorsCollection.fetchFromParam();

                this.layoutView.getRegion('content').show(motorsView);
            },
            messages: function () {
                this.init('messages');
                this.messagesView = new MessagesView();
                this.layoutView.getRegion('content').show(this.messagesView);
            },
            logs: function () {
                this.init('logs');
                this.logCollection = new App.Entities.LogCollection();
                this.logCollection.fetch();

                this.logsView = new LogsView({collection: this.logCollection});
                this.layoutView.getRegion('content').show(this.logsView);
            },
            processes: function () {
                this.init('processes');
                this.speedView = new SpeedView();
                this.layoutView.getRegion('content').show(this.speedView);
            },
            speed: function () {
                this.init('speed');
                this.speedView = new SpeedView();
                this.layoutView.getRegion('content').show(this.speedView);
            },
            system: function () {
                var self = this;
                this.init('system');
                var rosNodeCollection = new App.Monitor.Entities.RosNodeCollection();
                rosNodeCollection.fetch();
                this.systemView = new SystemView({
                    collection: rosNodeCollection
                });
                this.layoutView.getRegion('content').show(this.systemView);
            }
        };
    });
