define(['application', './views/layout', 'lib/api', '../performances/views/layout'],
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
