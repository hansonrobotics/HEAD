define(['application', './views/layout', 'modules/motors/views/motors', './views/messages', './views/logs',
        './views/speed', './views/system', './views/processes', 'entities/motor', 'entities/log'],
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
                }

                this.layoutView.setActive(name);
            },
            motors: function () {
                var self = this;
                this.init('motors');
                this.motorsCollection = new App.Entities.MotorCollection();
                this.motorsCollection.fetchFromParam(function () {
                    self.motorsCollection.setMonitorInterval();
                });

                this.motorsView = new MotorsView({
                    collection: this.motorsCollection,
                    monitoring: true
                });
                this.layoutView.getRegion('content').show(this.motorsView);
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
                this.systemView = new SystemView();
                this.layoutView.getRegion('content').show(this.systemView);
                $.ajax({
                    url: "/monitor/status",
                    dataType: "json",
                    success: function (data) {
                       self.systemView.config = _.extend(self.systemView.config, data);
                       self.systemView.render();
                    }
                });
            }
        };
    });
