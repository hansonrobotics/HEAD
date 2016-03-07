define(['application', './views/layout', 'lib/api', 'entities/animation'],
    function (App, LayoutView, api) {
        return {
            admin_index: function () {
                api.disableInteractionMode();
                api.pointHead();
                this.layoutView = new LayoutView();
                App.LayoutInstance.setTitle('Animations');
                App.LayoutInstance.showAdminNav();
                App.LayoutInstance.getRegion('content').show(this.layoutView);
            }
        }
    });
