define(['marionette', 'underscore'], function (Marionette, _) {
    Marionette.Region.prototype.attachHtml = function(view){
        this.$el.hide();
        this.$el.html(view.el);
        this.$el.slideDown();
    };

    var render = Marionette.ItemView.prototype.render;
    Marionette.ItemView.prototype.render = function () {
        // run original render
        _.bind(render, this)();
        this.$el.hide().slideDown();
    };
});