define(['application', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in', 'lib/api', 'vendor/select2.min', 'jquery-ui'],
    function (App, template, FadeInRegion, api) {
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
                },
                ui: {
                    languageButton: '.app-language-select button'
                },
                events: {
                    'click @ui.languageButton': 'changeLanguage'
                },
                changeLanguage: function (e) {
                    var language = $(e.target).data('lang');

                    this.ui.languageButton.removeClass('active');
                    $(e.target).addClass('active');

                    api.setRobotLang(language);
                }
            });
        });
    });
