define(['./templates/status.tpl', 'lib/regions/fade_in'],
    function(template, FadeInRegion) {
        return Marionette.View.extend({
            template: template
        })
    })
