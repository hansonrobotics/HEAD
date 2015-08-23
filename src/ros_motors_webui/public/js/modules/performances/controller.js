define(['application', './views/layout'],
    function (App, LayoutView) {
        return {
            performances: function () {
                this.layoutView = new LayoutView();
                App.LayoutInstance.setTitle('Interactions and Performances');
                App.LayoutInstance.getRegion('content').show(this.layoutView);
            }
        };
    });
