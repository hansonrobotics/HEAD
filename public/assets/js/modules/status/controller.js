define(['application', './views/status', 'lib/api'],
    function (App, StatusView, api) {
        return {
            index: function () {
                var self = this;

                api.blenderMode.disable();
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
                api.setExpression("Neutral", 0);

                this.layoutView = new StatusView();

                App.LayoutInstance.setTitle('Status');
                App.LayoutInstance.getRegion('content').show(this.layoutView);
                App.LayoutInstance.showNav();

                $.ajax({
                    url: "/api/system/status",
                    dataType: "json",

                    success: function (data) {
                        var indicators = "";

                        $.each(data, function () {
                            indicators += '<li><span class="glyphicon ' +
                            (this.success ? "glyphicon-ok" : "glyphicon-remove") +
                            '" aria-hidden="true"></span>' +
                            '<strong>' + this.label + '</strong> (' + this.cmd + ')' +
                            '</li>';
                        });

                        self.layoutView.ui.indicators.html(indicators);
                    }
                });
            },
            admin_index: function () {
                this.index();
                App.LayoutInstance.showAdminNav();
            }
        }
    });
