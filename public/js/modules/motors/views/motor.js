define(["application", "tpl!./templates/motor.tpl", 'jquery-ui'],
    function (App, motorTpl) {
        App.module("Motors.Views", function (View, App, Backbone, Marionette, $, _) {
            View.Motor = Marionette.ItemView.extend({
                template: motorTpl,
                tagName: 'li',
                modelEvents: {
                    "change": "modelChanged"
                },
                // indicates that motor is fully configuredselected
                ui: {
                    slider: '.app-slider',
                    value: '.app-slider-value',
                    setMinButton: '.app-motors-set-min',
                    setMaxButton: '.app-motors-set-max',
                    setDefaultButton: '.app-motors-set-default',
                    hideOnEdit: '.app-motors-hide-on-edit',
                    showOnEdit: '.app-motors-show-on-edit',
                    dragHandle: '.app-motor-drag-handle',
                    labelLeft: '.app-slider-label-left',
                    labelright: '.app-slider-label-right',
                    labelLeftInput: 'input.app-motor-label',
                    sliderMinVal: '.app-slider-min-value',
                    sliderMaxVal: '.app-slider-max-value',
                    container: '.app-slider-container',
                    selectButton: '.app-select-motor-button',
                    selectButtonIcon: '.app-select-motor-button span',
                    anglesButton: '.app-angles-button',
                    calibrationButton: '.app-calibration-button'
                },
                events: {
                    'click @ui.setMinButton': 'setMin',
                    'click @ui.setMaxButton': 'setMax',
                    'click @ui.setDefaultButton': 'setDefault',
                    'keyup @ui.labelLeftInput': 'setLabelLeft',
                    'click @ui.selectButton': 'toggleSelect',
                    'click @ui.anglesButton': 'anglesSelected',
                    'click @ui.calibrationButton': 'calibrationSelected'

                },
                modelChanged: function () {
                    if (this.calibrationEditEnabled()) {
                        var calibration = this.model.get('calibration');

                        this.setSliderMin(820);
                        this.ui.sliderMinVal.text(calibration['min']);
                        this.setSliderMax(2175);
                        this.ui.sliderMaxVal.text(calibration['max']);

                        this.setSliderValue(calibration['init']);
                    } else {
                        var min = this.model.getDegrees('min');
                        var max = this.model.getDegrees('max');

                        this.ui.sliderMinVal.text(min + '°');
                        this.ui.sliderMaxVal.text(max + '°');

                        if (this.editEnabled()) {
                            this.setSliderMin(-90);
                            this.setSliderMax(90);
                        } else {
                            this.setSliderMin(min);
                            this.setSliderMax(max);
                        }

                        this.setSliderValue(this.model.getDegrees('value'), true);
                    }

                    this.ui.labelLeft.text(this.model.get('labelleft'));
                    this.ui.labelright.text(this.model.get('labelright'));

                    this.selected(this.model.selected());
                },
                setSliderValue: function (value, degrees) {
                    this.ui.value.text(value + (degrees ? '°' : ''));
                    this.ui.slider.slider('value', value);
                },
                setSliderMin: function (value) {
                    this.ui.slider.slider('option', 'min', value);
                },
                setSliderMax: function (value) {
                    this.ui.slider.slider('option', 'max', value);
                },
                onRender: function () {
                    var self = this;
                    $(this.ui.slider).slider({
                        range: "min",
                        value: this.model.getDegrees('default'),
                        min: this.model.getDegrees('min'),
                        max: this.model.getDegrees('max'),
                        slide: function (e, ui) {
                            if (self.ui.calibrationButton.hasClass('active'))
                                self.setSliderValue(ui.value);
                            else
                                self.model.setDegrees('value', ui.value);
                        }
                    });

                    // hide if doesn't have label
                    if (!this.model.get('labelleft')) this.$el.hide();

                    // show angles by default
                    this.anglesSelected();

                    // deselect by default
                    this.modelChanged();

                    // hide select buttons by default
                    this.showSelectButton(false);
                },
                setMin: function () {
                    // cloning so that model change event is triggered
                    var calibration = _.clone(this.model.get('calibration')),
                        value = this.ui.slider.slider('value');

                    if (this.calibrationEditEnabled()) {
                        if (calibration) {
                            calibration['min'] = calibration['calibration']['min_pulse'] = value;
                            this.model.set('calibration', calibration);
                        }
                    } else {
                        this.ui.sliderMinVal.text(value);
                        this.model.setDegrees('min', value);

                        if (calibration) {
                            calibration['calibration']['min_angle'] = value;
                            this.model.set('calibration', calibration);
                        }
                    }
                },
                setMax: function () {
                    // cloning so that model change event is triggered
                    var calibration = _.clone(this.model.get('calibration')),
                        value = this.ui.slider.slider('value');

                    if (this.calibrationEditEnabled()) {
                        if (calibration) {
                            calibration['max'] = calibration['calibration']['max_pulse'] = value;
                            this.model.set('calibration', calibration);
                        }
                    } else {
                        this.ui.sliderMaxVal.text(value);
                        this.model.setDegrees('max', value);

                        if (calibration) {
                            calibration['calibration']['max_angle'] = value;
                            this.model.set('calibration', calibration);
                        }
                    }
                },
                setDefault: function () {
                    var value = this.ui.slider.slider('value');

                    if (this.calibrationEditEnabled()) {
                        // cloning so that model change event is triggered
                        var calibration = _.clone(this.model.get('calibration'));

                        if (calibration) {
                            calibration['init'] = value;
                            this.model.set('calibration', calibration);
                        }
                    } else {
                        this.model.set('default', value);
                    }
                },
                setLabelLeft: function () {
                    var label = this.ui.labelLeftInput.val();
                    this.model.set('labelleft', label);
                    this.ui.labelLeft.html(label);
                },
                enableEdit: function () {
                    if (! this.editEnabled()) {
                        this.ui.dragHandle.show();

                        if (this.model.get('motor_id') != null) {
                            this.edit_enabled = true;

                            this.ui.showOnEdit.show();
                            this.ui.hideOnEdit.hide();
                        }

                        this.$el.show();
                    }
                },
                editEnabled: function () {
                    return typeof this.edit_enabled != 'undefined' && this.edit_enabled;
                },
                calibrationEditEnabled: function () {
                    return this.ui.calibrationButton.hasClass('active');
                },
                /**
                 * Sets select model select state if boolean argument is present,
                 * returns current state (defaults to false) otherwise
                 *
                 * @param selected
                 * @returns {boolean}
                 */
                selected: function (selected) {
                    if (typeof selected == 'undefined') {
                        return this.model.selected();
                    } else {
                        if (selected) {
                            this.ui.selectButton.addClass('btn-success');
                            this.ui.selectButton.removeClass('btn-danger');

                            this.ui.selectButtonIcon.addClass('glyphicon-ok');
                            this.ui.selectButtonIcon.removeClass('glyphicon-remove');
                        } else {
                            this.ui.selectButton.removeClass('btn-success');
                            this.ui.selectButton.addClass('btn-danger');

                            this.ui.selectButtonIcon.removeClass('glyphicon-ok');
                            this.ui.selectButtonIcon.addClass('glyphicon-remove');
                        }
                    }
                },
                toggleSelect: function () {
                    this.model.selected() ? this.model.selected(false) : this.model.selected(true);
                },
                showSelectButton: function (show) {
                    if (show) {
                        this.ui.selectButton.show();
                    } else {
                        this.ui.selectButton.hide();
                    }
                },
                calibrationSelected: function () {
                    this.ui.calibrationButton.addClass('active');
                    this.ui.anglesButton.removeClass('active');

                    this.modelChanged();
                },
                anglesSelected: function () {
                    this.ui.anglesButton.addClass('active');
                    this.ui.calibrationButton.removeClass('active');

                    this.modelChanged();
                }
            });
        });

        return App.module('Motors.Views.Motor');
    });
