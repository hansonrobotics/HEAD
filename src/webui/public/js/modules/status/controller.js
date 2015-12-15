define(['application', './views/status_list', 'lib/api', './entities/status_collection'],
    function (App, StatusListView, api, StatusCollection) {
        return {
            index: function () {
                var statusCollection = new StatusCollection(),
                    statusListView = new StatusListView({collection: statusCollection});

                api.blenderMode.disable();
                api.disableInteractionMode();
                api.setExpression("Neutral", 0);

                statusCollection.fetch();

                App.LayoutInstance.setTitle('Status');
                App.LayoutInstance.getRegion('content').show(statusListView);
                App.LayoutInstance.showNav();

                //$.ajax({
                //    url: "/status",
                //    dataType: "json",
                //
                //    success: function (data) {
                //        var indicators = "";
                //
                //        $.each(data, function () {
                //            indicators += '<li><span class="glyphicon ' +
                //                (this.success ? "glyphicon-ok" : "glyphicon-remove") +
                //                '" aria-hidden="true"></span>' +
                //                '<strong>' + this.label + '</strong> (' + this.cmd + ')' +
                //                '</li>';
                //        });
                //
                //        self.layoutView.ui.indicators.html(indicators);
                //    }
                //});
            },
            admin_index: function () {
                this.index();
                App.LayoutInstance.showAdminNav();
            }
        }
    });
