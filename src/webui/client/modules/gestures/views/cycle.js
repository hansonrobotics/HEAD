define(["marionette", "./templates/cycle.tpl", 'lib/api'], function (Marionette, template, api) {
    return Marionette.View.extend({
        tagName: 'span',
        template: template,
        ui: {
            buttonOn: '.app-cycle-button',
            buttonOff: '.app-cycle-off-button'
        },
        events: {
            'click @ui.buttonOn': 'cycleClicked',
            'click @ui.buttonOff': 'cycleOffClicked'
        },
        cycleClicked: function () {
            var button = this.ui.button;
            $(button).addClass('active');
            api.setSomaState(this.model.get('name'));
        },
        cycleOffClicked: function () {
            var button = this.ui.button;
            $(button).addClass('active');
            api.setSomaState(this.model.get('name'), 1.0, 0.0);
        }
    });
});
