define(['application', 'tpl!./templates/ros_node.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.RosNode = Marionette.ItemView.extend({
            template: template
        });
        return App.module('Monitor.Views').RosNode;
    });
 });