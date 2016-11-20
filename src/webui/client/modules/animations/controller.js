define(['application', './views/layout', 'lib/api', './css/animations'],
    function (App, LayoutView, api) {
        return {
            admin_index: function () {
                api.disableInteractionMode();
                api.blenderMode.disable();
                api.setExpression("Neutral", 0);
                api.pointHead();
                this.layoutView = new LayoutView();
                App.LayoutInstance.setTitle('Animations');
                App.LayoutInstance.showAdminNav();
                App.LayoutInstance.getRegion('content').show(this.layoutView);
            }
        }
    });
