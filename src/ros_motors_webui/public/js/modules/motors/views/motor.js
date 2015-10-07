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
                this.setSliderValue(this.model.get('value'));

                this.ui.sliderMinVal.text(this.model.get('min')/* + '°'*/);
                this.ui.sliderMaxVal.text(this.model.get('max')/* + '°'*/);
                this.ui.labelLeft.text(this.model.get('labelleft') || this.model.get('name'));
                this.ui.labelright.text(this.model.get('labelright'));

                var indicatorStatus;
                $(this.ui.indicator).removeClass('app-ok app-error app-inactive');

                switch (this.model.get('error')) {
                    case 0:
                        indicatorStatus = 'ok';
                        $(this.ui.indicator).addClass('app-ok');
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

                $(this.ui.indicator).popover({content: indicatorStatus, trigger: 'hover'});
            },
            setSliderValue: function (value, degrees) {
                if (degrees) {
                    this.ui.value.text(value + (degrees ? '°' : ''));
                }
                this.ui.value.text(value + (degrees ? '°' : ''));
                this.ui.slider.slider('value', value);
            },
            onRender: function () {
                var self = this;
                $(this.ui.slider).slider({
                    range: "min",
                    value: this.model.get('init'),
                    min: this.model.get('min'),
                    max: this.model.get('max'),
                    slide: function (e, ui) {
                        self.model.set('value', ui.value)
                    }
                });

                // deselect by default
                this.modelChanged();
            }
        });
    });
