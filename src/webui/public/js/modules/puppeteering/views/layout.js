define(['marionette', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in'],
    function (Marionette, template, FadeInRegion) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {},
            events: {},
            regions: {
                timeline: {
                    el: ".app-timeline-editor-region",
                    regionClass: FadeInRegion
                }
            }
        });
    });