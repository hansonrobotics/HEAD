define(['application', './views/dashboard', 'lib/api', '../performances/views/layout', './css/puppeteering'],
    function (App, LayoutView, api) {
        return {
            index: function () {
                var layoutView = new LayoutView();

                api.blenderMode.enable();
                api.disableInteractionMode();

                App.LayoutInstance.getRegion('content').show(layoutView);
                App.LayoutInstance.setTitle('Puppeteering');
                App.LayoutInstance.setFluid(true);
            }
        };
    });
