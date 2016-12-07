define(['application', './templates/layout.tpl', "modules/motors/views/motors", './expressions',
    'entities/motor_collection', 'entities/expression_collection', 'scrollbar', 'scrollbar-css'],
    function (App, template, MotorsView, ExpressionsView, MotorCollection, ExpressionCollection) {
    return Marionette.View.extend({
        template: template,
        className: 'app-expressions-admin',
        ui: {
            motors: ".app-motors",
            expressions: ".app-expressions"
        },
        regions: {
            motors: ".app-motors",
            expressions: ".app-expressions"
        },
        onRender: function () {
            var motorsCollection = new MotorCollection(),
                motorsView = new MotorsView({collection: motorsCollection});

            this.getRegion('motors').show(motorsView);
            motorsCollection.fetchFromParam(function () {
                motorsCollection.setDefaultValues();
            });

            var expressions = new ExpressionCollection(),
                expressionsView = new ExpressionsView({
                    collection: expressions,
                    motors: motorsCollection
                });

            expressions.fetch();
            this.getRegion('expressions').show(expressionsView);
        },
        onAttach: function () {
            var self = this,
                updateDimensions = function () {
                    if (self.isDestroyed())
                        $(window).off('resize', updateDimensions);
                    else
                        self.updateDimensions();
                };

            this.ui.expressions.perfectScrollbar({suppressScrollX: true});
            this.ui.motors.perfectScrollbar({suppressScrollX: true});
            $(window).resize(updateDimensions).resize();
        },
        updateDimensions: function () {
            var height = App.LayoutInstance.getContentHeight();

            if (this.ui.motors.offset().left == this.ui.expressions.offset().left)
                height = 'auto';

            this.ui.motors.height(height).perfectScrollbar('update');
            this.ui.expressions.height(height).perfectScrollbar('update');
        }
    });
});