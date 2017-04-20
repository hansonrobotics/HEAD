define(["marionette", "./status", 'tpl!./templates/status_list.tpl'],
    function (Marionette, childView, template) {
        return Marionette.CompositeView.extend({
            childView: childView,
            childViewContainer: '.app-status-indicators',
            template: template
        });
    });
