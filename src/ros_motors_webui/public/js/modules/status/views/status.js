define(["marionette", "tpl!./templates/status.tpl"], function (Marionette, template) {
    return Marionette.ItemView.extend({
        template: template,
        tagName: 'li'
    });
});
