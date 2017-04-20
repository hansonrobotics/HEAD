define(['jquery', 'marionette', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in'],
    function ($, Marionette, template, FadeInRegion) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                nav: '.app-nav',
                motors: '.app-motors-link',
                messages: '.app-messages-link',
                logs: '.app-logs-link',
                speed: '.app-speed-link',
                system: '.app-system-link'
            },
            events: {
            },
            regions: {
                content: {
                    el: '.app-content',
                    regionClass: FadeInRegion
                }
            },
            setActive: function (name) {
                this.ui.nav.find('li').removeClass('active');
                this.ui[name].addClass('active');
            }
        });
    });
