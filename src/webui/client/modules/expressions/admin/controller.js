define(["application", "lib/api", "./views/layout"],
    function (App, api, LayoutView) {
        return {
            index: function () {
                // reset robot
                api.disableInteractionMode();
                api.blenderMode.disable();
                api.setExpression("Neutral", 0);
                api.pointHead();

                var layoutView = new LayoutView();

                App.LayoutInstance.showAdminNav();
                App.LayoutInstance.setTitle('Motors');
                App.LayoutInstance.getRegion('content').show(layoutView);
            }
        };
    });