define(['application', './views/layout', './css/gestures'],
    function (App, LayoutView) {
        var gestures = {
            index: function () {
                this.layoutView = new LayoutView();

                App.LayoutInstance.setTitle('Gestures');
                App.LayoutInstance.getRegion('content').show(this.layoutView);
                App.LayoutInstance.showNav();
            }
        };

        return gestures;
    });
