define(['application', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in', 'vendor/select2.min', 'jquery-ui'],
    function (App, template, FadeInRegion) {
        App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
            Views.Layout = Marionette.LayoutView.extend({
                template: template,
                regions: {
                    performances: {
                        el: '.app-performances-region',
                        regionClass: FadeInRegion
                    },
                    timeline: {
                        el: '.app-timeline-region',
                        regionClass: FadeInRegion
                    },
                    queue: {
                        el: '.app-performance-queue-container',
                        regionClass: FadeInRegion
                    }
                }
            });
        });
    });
