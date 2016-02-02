define(['application', 'lib/api', './views/layout'],
    function (App, api, LayoutView) {
        return {
            performances: function () {
                var layoutView = new LayoutView();

                api.blenderMode.enable();
                api.disableInteractionMode();

                // show page
                App.LayoutInstance.setTitle('Interactions and Performances');
                App.LayoutInstance.getRegion('content').show(layoutView);
            }
        };
    });
