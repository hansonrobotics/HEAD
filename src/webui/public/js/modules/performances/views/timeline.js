define(['marionette', 'tpl!./templates/timeline.tpl'], function (Marionette, template) {
    return Marionette.ItemView.extend({
        template: template,
        className: 'app-timeline-container',
        ui: {
            checkbox: '.app-timeline-checkbox input',
            nodes: '.app-timeline-nodes'
        }
    });
});
