define(['application', 'lib/api', './views/layout', './views/attention_regions'],
    function (App, api, LayoutView, AttentionRegionsView) {
        return {
            performances: function () {
                var layoutView = new LayoutView();

                api.blenderMode.enable();
                api.disableInteractionMode();

                // show page
                App.LayoutInstance.setTitle('Interactions and Performances');
                App.LayoutInstance.setFluid(true);
                App.LayoutInstance.getRegion('content').show(layoutView);
            },
            attention_regions: function () {
                var attentionRegionsView = new AttentionRegionsView();
                api.disableInteractionMode();

                // show page
                App.LayoutInstance.setTitle('Attention Regions');
                App.LayoutInstance.setFluid(false);
                App.LayoutInstance.getRegion('content').show(attentionRegionsView);
            }
        };
    });
