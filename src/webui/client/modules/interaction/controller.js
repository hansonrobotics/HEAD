define(['application', './views/interaction', 'lib/api', './entities/face_collection', './entities/message_collection'],
    function (App, InteractionView, api, FaceCollection, MessageCollection) {
        return {
            index: function () {
                api.blenderMode.enable();
                api.enableInteractionMode();
                App.LayoutInstance.setTitle('Interaction');
                App.LayoutInstance.showNav();
                App.LayoutInstance.setFluid(false);

                var interactionView = new InteractionView();
                App.LayoutInstance.getRegion('content').show(interactionView);
            }
        };
    });
