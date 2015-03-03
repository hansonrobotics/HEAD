define(["ros_ui", "tpl!./templates/motor.tpl"], function (UI, motorTpl) {
    UI.module("Motors.View", function (View, RosUI, Backbone, Marionette, $, _) {
        View.Motor = Marionette.ItemView.extend({
            template: motorTpl,
            tagName: 'li',
            modelEvents: {
                "change": "modelChanged"
            },
            // indicates that motor is fully configured
            motor_configured: true,
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
                selectButtonIcon: '.app-select-motor-button span'
            },
            events: {
                'click @ui.setMinButton': 'setMin',
                'click @ui.setMaxButton': 'setMax',
                'click @ui.setDefaultButton': 'setDefault',
                'keyup @ui.labelLeftInput': 'setLabelLeft',
                'click @ui.selectButton': 'toggleSelect'
            },
            initialize: function () {
                this.setMotorConfigured();
            },
            // passes data to the template
            serializeData: function () {
                return _.extend(this.model.toJSON(), {
                    min: this.model.getMinDeg(),
                    max: this.model.getMaxDeg()
                });
            },
            modelChanged: function () {
                this.setMotorConfigured();
                this.ui.value.text(this.model.getValueDeg());
                this.ui.slider.slider('value', this.model.getValueDeg());
                this.ui.labelLeft = this.model.get('labelleft');
                this.ui.labelright = this.model.get('labelright');
                this.ui.min = this.model.get('min');
                this.ui.max = this.model.get('max');

                // select updated motor
                this.select(true);
            },
            setMotorConfigured: function () {
                // motor is considered configured if it has label defined
                this.motor_configured = !!this.model.get('labelleft');
            },
            getMotorConfigured: function () {
                return this.motor_configured;
            },
            onRender: function () {
                var self = this;
                $(this.ui.slider).slider({
                    range: "min",
                    value: this.model.getDefaultDeg(),
                    min: this.model.getMinDeg(),
                    max: this.model.getMaxDeg(),
                    slide: function (e, ui) {
                        self.model.setValueDeg(ui.value);
                    },
                    start: function (e, ui) {
                        self.model.set('isActive', true);
                    },
                    stop: function (e, ui) {
                        self.model.set('isActive', false);
                    }
                });
                if (!this.getMotorConfigured()) this.$el.hide();
                // deselect by default
                this.select(false);
            },
            setMin: function () {
                var valueDeg = this.model.getValueDeg();
                this.ui.sliderMinVal.html(valueDeg);
                this.model.setMinDeg(valueDeg);
            },
            setMax: function () {
                var valueDeg = this.model.getValueDeg();
                this.ui.sliderMaxVal.html(valueDeg);
                this.model.setMaxDeg(valueDeg);
            },
            setDefault: function () {
                this.model.setDefaultDeg(this.model.getValueDeg());
            },
            setLabelLeft: function () {
                var label = this.ui.labelLeftInput.val();
                this.model.set('labelleft', label);
                this.ui.labelLeft.html(label);
            },
            enableEdit: function () {
                this.ui.dragHandle.show();

                if (this.model.get('editable') == true) {
                    this.ui.showOnEdit.show();
                    this.ui.hideOnEdit.hide();
                }

                this.$el.show();
            },
            disableEdit: function () {
                this.ui.dragHandle.hide();

                if (this.model.get('editable') == true) {
                    this.ui.showOnEdit.hide();
                    this.ui.hideOnEdit.show();
                }

                if (!this.getMotorConfigured()) this.$el.hide();
            },
            /**
             * Sets select state if boolean argument is present,
             * returns current state if no argument provided
             *
             * @param selected
             * @returns {boolean}
             */
            select: function (selected) {
                if (typeof selected == 'undefined') {
                    return typeof this.selected == 'undefined' ? false : this.selected;
                } else {
                    this.selected = selected;

                    if (this.selected) {
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
                if (typeof this.selected == 'undefined' || !this.selected) {
                    this.select(true);
                } else {
                    this.select(false);
                }
            }
        });
    });

    return UI.Motors.View.Motor;
});
