define(['application', './views/layout', 'lib/api', '../performances/views/layout'],
    function (App, LayoutView, api, TimelineEditorView) {
        return {
            index: function () {
                var layoutView = new LayoutView(),
                    timelineEditorView = new TimelineEditorView({fluid: true});

                api.blenderMode.enable();
                api.disableInteractionMode();

                App.LayoutInstance.getRegion('content').show(layoutView);
                App.LayoutInstance.setTitle('Interactions and Performances');

                layoutView.getRegion('timeline').show(timelineEditorView);
            }
        };
    });
