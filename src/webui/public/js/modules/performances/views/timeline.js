define(['marionette', 'tpl!./templates/timeline.tpl', '../entities/node'], function (Marionette, template, Node) {
    return Marionette.ItemView.extend({
        template: template,
        className: 'app-timeline-container',
        ui: {
            checkbox: '.app-timeline-checkbox input',
            nodes: '.app-timeline-nodes'
        },
        initialize: function (options) {
            this.mergeOptions(options, ['performance', 'config']);
        },
        onRender: function () {
            var self = this;
            this.ui.nodes.droppable({
                accept: '.app-node[data-name]',
                tolerance: 'touch',
                drop: function (e, ui) {
                    var nodeEl = $(ui.helper),
                        node = self.performance.get('nodes').get({cid: nodeEl.data('cid')}),
                        startTime = Math.round(
                            (ui.offset.left - self.ui.nodes.offset().left) / self.config.pxPerSec * 100
                        ) / 100;

                    if (node)
                        node.set('start_time', startTime);
                    else
                        self.performance.get('nodes').add({
                            name: nodeEl.data('name'),
                            start_time: startTime,
                            duration: 1
                        });
                }
            });
        }
    });
});
