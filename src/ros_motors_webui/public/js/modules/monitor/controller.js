define(['application', './views/layout', 'modules/motors/views/motors', './views/messages', './views/logs',
        './views/speed', './views/system', './views/processes', 'entities/motor', ],
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
                this.init('motors');
                this.motorsCollection = new App.Entities.MotorCollection();
                this.motorsCollection.fetch();

                this.motorsView = new MotorsView({collection: this.motorsCollection});
                this.layoutView.getRegion('content').show(this.motorsView);
            },
            messages: function () {
                this.init('messages');
                this.messagesView = new MessagesView();
                this.layoutView.getRegion('content').show(this.messagesView);
            },
            logs: function () {
                this.init('logs');
                this.logsView = new LogsView();
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
                this.init('system');
                this.systemView = new SystemView();
                this.layoutView.getRegion('content').show(this.systemView);
            }
        };
    });
