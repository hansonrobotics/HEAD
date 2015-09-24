define(['application', './views/interaction', 'lib/api', 'entities/interaction'],
    function (App, InteractionView, api) {
        return {
            index: function () {
                var messageCollection = new App.Entities.MessageCollection();
                this.interactionView = new InteractionView({collection: messageCollection});

                App.LayoutInstance.setTitle('Interaction');
                App.LayoutInstance.getRegion('content').show(this.interactionView);
                App.LayoutInstance.showNav();
            }
        };
    });
