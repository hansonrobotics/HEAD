define(["application", "lib/api", "./views/layout", "../../motors/views/motors", './views/expressions',
        'entities/motor', 'entities/expression'],
    function (App, api, LayoutView, MotorsView, ExpressionsView) {
        return {
            index: function () {
                // reset robot
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
                api.pointHead();

                this.motorsCollection = new App.Entities.MotorCollection();
                this.layoutView = new LayoutView();
                this.motorsView = new MotorsView({collection: this.motorsCollection});

                App.LayoutInstance.showAdminNav();
                App.LayoutInstance.setTitle('Motors');
                App.LayoutInstance.getRegion('content').show(this.layoutView);
                this.layoutView.getRegion('motors').show(this.motorsView);

                this.motorsCollection.fetch();
                var expressions = new App.Entities.ExpressionCollection(),
                    expressionsView = new ExpressionsView({
                        collection: expressions,
                        motors: this.motorsCollection,
                        controller: this
                    });

                expressions.fetch();
                this.layoutView.getRegion('expressions').show(expressionsView);
            }
        };
    });