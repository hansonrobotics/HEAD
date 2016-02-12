define(['marionette', 'tpl!./templates/faces.tpl'], function (Marionette, template) {
    return Marionette.ItemView.extend({
        template: template
    });
});