define(['application', 'tpl!./templates/timeline.tpl', 'jquery-ui', '../entities/node'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Timeline = Marionette.ItemView.extend({
            template: template,
            className: 'app-timeline-container',
            ui: {
                checkbox: '.app-timeline-checkbox input',
                nodes: '.app-timeline-nodes'
            },
            onRender: function () {
                //var self = this;
                //
                //$(this.ui.nodes).droppable({
                //    tolerance: 'intersect',
                //    drop: function (event, ui) {
                //        self.addNode($(ui.helper));
                //    }
                //});
            },
            selected: function (value) {
                if (typeof value == 'undefined')
                    return this.ui.checkbox.prop('checked');
                else {
                    this.ui.checkbox.prop('checked', value);
                }
            }
            //addNode: function (el) {
            //    var name = $(el).data('name'),
            //        nodeModel = null;
            //
            //    if (!$(el).hasClass('app-copied')) {
            //        el = $(el).clone().draggable({
            //            revert: 'invalid',
            //            snap: '.app-timeline-nodes',
            //            stop: function () {
            //                $(this).draggable('option', 'revert', 'invalid');
            //            }
            //        }).addClass('app-copied');
            //
            //        nodeModel = new App.Performances.Entities.Node({
            //            name: name,
            //            el: el
            //        });
            //
            //        nodeModel.on('change', function () {
            //            $(this.get('el')).animate({
            //                left: this.get('offset') * 100,
            //                width: this.get('duration') * 100
            //            });
            //        });
            //
            //        //el.data('model', nodeModel);
            //        this.model.get('nodes').add(nodeModel);
            //        console.log('here');
            //        return nodeModel;
            //    } else {
            //        nodeModel = $(el).data('model');
            //    }
            //
            //    this.ui.nodes.append(el);
            //    return nodeModel;
            //}
        });
    });
});
