define(['application', './views/layout', 'lib/api', '../performances/views/layout', './css/puppeteering'],
    function (App, LayoutView, api) {
        return {
            index: function () {
                var layoutView = new LayoutView();
                api.blenderMode.enable();
                App.LayoutInstance.getRegion('content').show(layoutView);
                App.LayoutInstance.setTitle('Puppeteering');
                App.LayoutInstance.setFluid(true);
            }
        };
    });
