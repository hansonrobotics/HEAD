define(["marionette", "tpl!./templates/motor.tpl", 'jquery', 'jquery-ui'],
    function (Marionette, template, $) {
        return Marionette.ItemView.extend({
            template: template,
            className: 'app-slider-container',
            modelEvents: {
                "change": "modelChanged"
            },
            ui: {
                slider: '.app-slider',
                value: '.app-slider-value',
                labelLeft: '.app-slider-label-left',
                labelright: '.app-slider-label-right',
                sliderMinVal: '.app-slider-min-value',
                sliderMaxVal: '.app-slider-max-value',
                indicator: '.app-motor-status-indicator'
            },
            serializeData: function () {
                var model = this.model.toJSON();

                if (typeof this.options.motorGroupLabel != 'undefined' && this.options.motorGroupLabel) {
                    return _.extend(model, {
                        groupLabel: this.options.motorGroupLabel
                    });
                } else
                    return model;
            },
            modelChanged: function () {
                this.ui.value.text(this.model.getDegrees('value') + '°');
                this.ui.sliderMinVal.text(this.model.getDegrees('min') + '°');
                this.ui.sliderMaxVal.text(this.model.getDegrees('max') + '°');
                this.ui.labelLeft.text(this.model.get('labelleft') || this.model.get('name'));
                this.ui.labelright.text(this.model.get('labelright'));

                if (this.options['monitoring']) {
                    var indicatorStatus;

                    $(this.ui.slider).addClass('app-monitoring');
                    $(this.ui.indicator).removeClass('app-ok app-error app-inactive');

                    switch (this.model.get('error')) {
                        case 0:
                            $(this.ui.indicator).addClass('app-ok');
                            indicatorStatus = 'ok';
                            break;
                        case 1:
                            $(this.ui.indicator).addClass('app-error');
                            indicatorStatus = 'error';
                            break;
                        case 2:
                            $(this.ui.indicator).addClass('app-inactive');
                            indicatorStatus = 'not available';
                            break;
                    }


                    $(this.ui.indicator).attr('data-content', indicatorStatus).popover({trigger: 'hover'});
                } else {
                    $(this.ui.slider).removeClass('app-monitoring');
                    $(this.ui.indicator).hide();
                }
            },
            onRender: function () {
                var self = this;
                $(this.ui.slider).slider({
                    disabled: !!this.options.monitoring,
                    range: "min",
                    value: this.model.getDegrees('default'),
                    min: this.model.getDegrees('min'),
                    max: this.model.getDegrees('max'),
                    slide: function (e, ui) {
                        self.model.setDegrees('value', ui.value)
                    }
                });

                // deselect by default
                this.modelChanged();
            }
        });
    });
