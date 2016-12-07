define(['application', './views/layout', './views/logs', './views/system', './entities/log_collection',
        './entities/check_collection', './css/status'],
    function (App, LayoutView, LogsView, SystemView, LogCollection, CheckCollection) {
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
                this.logCollection = new LogCollection();
                this.logCollection.fetch();

                this.logsView = new LogsView({collection: this.logCollection});
                this.layoutView.getRegion('content').show(this.logsView);
            },
            system: function () {
                this.init('system');
                var checksCollection = new CheckCollection();
                checksCollection.fetch();
                this.systemView = new SystemView({
                    collection: checksCollection
                });
                this.layoutView.getRegion('content').show(this.systemView);
            }
        };
    });
