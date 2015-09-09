define(['application', 'tpl!./templates/performances.tpl', './performance'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Performances = Marionette.CompositeView.extend({
            template: template,
            childView: App.Performances.Views.Performance,
            childViewContainer: '.app-performances',
            ui: {
                newButton: '.app-new'
            },
            events: {
                'click @ui.newButton': 'addNew'
            },
            addChild: function(child, ChildView, index){
                Marionette.CollectionView.prototype.addChild.apply(this, arguments);
            },
            addNew: function () {
                var performance = new App.Performances.Entities.Performance({name: 'New performance'});
                this.collection.add(performance);
                Views.trigger('performance:add', performance);
            }
        });
    });
});
