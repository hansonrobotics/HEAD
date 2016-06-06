define(['application', './views/layout', './views/logs',
        './views/system', './entities/log', './entities/checks'],
    function (App, LayoutView, LogsView, SystemView) {
        return {
            init: function (name) {
                if (!this.layoutView) {
                    var self = this;

                    this.layoutView = new LayoutView();
                    this.layoutView.on('destroy', function () {
                        delete self.layoutView;
                    });

                    App.LayoutInstance.setTitle('Hanson Robotics');
                    App.LayoutInstance.getRegion('content').show(this.layoutView);
                    App.LayoutInstance.hideNav();
                }

                this.layoutView.setActive(name);
            },
            logs: function () {
                this.init('logs');
                this.logCollection = new App.Entities.LogCollection();
                this.logCollection.fetch();

                this.logsView = new LogsView({collection: this.logCollection});
                this.layoutView.getRegion('content').show(this.logsView);
            },
            system: function () {
                var self = this;
                this.init('system');
                var checksCollection = new App.Start.Entities.CheckCollection();
                checksCollection.fetch();
                this.systemView = new SystemView({
                    collection: checksCollection
                });
                this.layoutView.getRegion('content').show(this.systemView);
            }
        };
    });
