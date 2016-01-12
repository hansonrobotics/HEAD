define(['application', './views/interaction', 'lib/api', './entities/face_collection', 'entities/interaction'],
    function (App, InteractionView, api, FaceCollection) {
        return {
            index: function () {
                api.blenderMode.enable();
                App.LayoutInstance.setTitle('Interaction');
                App.LayoutInstance.showNav();

                var messageCollection = new App.Entities.MessageCollection(),
                    faces = new FaceCollection(),
                    interactionView = new InteractionView({collection: messageCollection, faceCollection: faces});

                App.LayoutInstance.getRegion('content').show(interactionView);
            }
        };
    });
