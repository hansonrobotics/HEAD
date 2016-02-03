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
                indicator: '.app-motor-status-indicator',
                dynamixelButtonTooltip: '.app-dynamixel-button-tooltip',
                dynamixelParams: '.app-dynamixel-params',
                selectButton: '.app-select-motor-button',
                selectButtonIcon: '.app-select-motor-button span'
            },
            events: {
                'click @ui.selectButton': 'toggleSelect'
            },
            serializeData: function () {
                var data = _.extend(this.model.toJSON(), {
                    uniqueId: this.model.cid
                });

                if (typeof this.options.motorGroupLabel != 'undefined' && this.options.motorGroupLabel) {
                    return _.extend(data, {
                        groupLabel: this.options.motorGroupLabel
                    });
                } else
                    return data;
            },
            modelChanged: function () {
                this.ui.value.text(this.model.getDegrees('value') + '°');
                this.ui.sliderMinVal.text(this.model.getDegrees('min') + '°');
                this.ui.sliderMaxVal.text(this.model.getDegrees('max') + '°');
                this.ui.labelLeft.text(this.model.get('labelleft') || this.model.get('name'));
                this.ui.labelright.text(this.model.get('labelright'));

                if (this.options['monitoring']) {
                    $(this.ui.slider).slider('value', 0);
                    this.ui.value.text('0°');
                }

                if (this.options['monitoring'] && this.model.get('status')) {
                    this.enableMonitoring();
                } else {
                    this.disableMonitoring();
                }

                this.selected(this.model.selected());
            },
            enableMonitoring: function () {
                var self = this,
                    indicatorStatus,
                    status = this.model.get('status');

                // set value
                if (status.position) {
                    $(this.ui.slider).slider('value', status.position);
                    this.ui.value.text(status.position + '°');
                }

                // show indicator
                $(this.ui.slider).addClass('app-monitoring');
                $(this.ui.indicator).show().removeClass('app-ok app-error app-inactive');

                switch (status.error) {
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

                // show indicator tooltip
                $(this.ui.indicator).tooltip({
                    title: indicatorStatus
                });

                if (status['hardware'] == 'dynamixel' && status['motor_state']) {
                    // show dynamixel collapse button
                    this.ui.dynamixelButtonTooltip.show().tooltip({trigger: 'hover'});

                    // show dynamixel params
                    this.ui.dynamixelParams.html('');
                    $.each(status['motor_state'], function (key, value) {
                        $(self.ui.dynamixelParams).append($('<dt>').html(key)).append($('<dd>').html(value))
                    });
                }
            },
            disableMonitoring: function () {
                this.ui.dynamixelButtonTooltip.hide();
                $(this.ui.slider).removeClass('app-monitoring');
                $(this.ui.indicator).hide();
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

                // hide select buttons by default
                this.showSelectButton(false);
            },
            showSelectButton: function (show) {
                if (show) {
                    this.ui.selectButton.show();
                } else {
                    this.ui.selectButton.hide();
                }
            },
            toggleSelect: function () {
                this.model.selected() ? this.model.selected(false) : this.model.selected(true);
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
        });
    });
