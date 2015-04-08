define(["application", "lib/api", './views/motors', './views/layout', 'entities/motor'],
    function (App, api, MotorsView, LayoutView) {
        return {
            public_index: function () {
                // reset robot
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
                api.pointHead();

                App.LayoutInstance.showNav();
                this.show_motors(false);
            },
            admin_index: function () {
                App.LayoutInstance.showAdminNav();
                this.show_motors(true);
            },
            show_motors: function (enableEdit) {
                this.motorsCollection = new App.Entities.MotorCollection();
                this.motorsCollection.fetch();

                this.motorsView = new MotorsView({collection: this.motorsCollection, enable_edit: enableEdit});
                this.layoutView = new LayoutView();

                App.LayoutInstance.setTitle('Motors');
                App.LayoutInstance.getRegion('content').show(this.layoutView);

                this.layoutView.getRegion('motors').show(this.motorsView);
            }

        };
    });
