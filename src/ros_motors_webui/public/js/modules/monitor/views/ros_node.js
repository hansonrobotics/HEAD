define(['application', 'tpl!./templates/ros_node.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.RosNode = Marionette.ItemView.extend({
            template: template,
            tagName: 'div',
            className: 'col-xs-6',
            status_cls: 'label-warning',
            status_label: 'n/a',
            ui: {
                status: '.app-status'
            },
            onRender: function () {
                if (this.model.get('status')  == 0){
                    this.status_cls = 'label-success';
                    this.status_label = 'OK';
                }
                if (this.model.get('status') == 1){
                    this.status_cls = 'label-danger';
                    this.status_label = 'Err';
                }
                this.ui.status.text(this.status_label).addClass(this.status_cls);
            }
        });
        return App.module('Monitor.Views').RosNode;
    });
 });