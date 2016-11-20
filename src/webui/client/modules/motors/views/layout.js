define(['application', './templates/layout.tpl'], function (App, template) {
    return Marionette.View.extend({
        template: template,

        regions: {
            motors: ".app-motors"
        }
    });
});