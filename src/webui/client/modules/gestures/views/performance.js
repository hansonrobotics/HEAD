define(["marionette", "./templates/performance.tpl", 'lib/api'],
    function (Marionette, template, api) {
        return Marionette.View.extend({
            tagName: 'span',
            template: template,
            ui: {
                button: 'button'
            },
            events: {
                'click @ui.button': 'performanceClicked'
            },
            performanceClicked: function () {
                var button = this.ui.button;
                $(button).addClass('active');

                api.executeScript(this.model.get('name'));

                setTimeout(function () {
                    $(button).removeClass('active');
                    $(button).blur();
                }, 500)
            }
        });
    });
