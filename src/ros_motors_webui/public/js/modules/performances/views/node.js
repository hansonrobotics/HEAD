define(['application', 'tpl!./templates/node.tpl', 'lib/api', 'lib/utilities', 'jquery-ui', 'lib/crosshair-slider',
        'vendor/select2.min'],
    function (App, template, api, utilities) {
        App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
            Views.Node = Marionette.ItemView.extend({
                template: template,
                ui: {
                    magnitudeSlider: '.app-magnitide-slider',
                    emotionSelect: 'select.app-emotion-select',
                    gestureSelect: 'select.app-gesture-select',
                    textInput: '.app-node-text',
                    startTime: '.app-node-start-time',
                    duration: '.app-node-duration',
                    crosshair: '.app-crosshair',
                    deleteButton: '.app-delete-node-button',
                    speedSlider: '.app-speed-slider',
                    speedLabel: '.app-speed-label',
                    magnitudeLabel: '.app-magnitude-label'
                },
                events: {
                    'change @ui.duration': 'setDuration',
                    'change @ui.startTime': 'setStartTime',
                    'keyup @ui.textInput': 'setText',
                    'change @ui.textInput': 'setTextDuration',
                    'change @ui.emotionSelect': 'setEmotion',
                    'change @ui.gestureSelect': 'setGesture',
                    'click @ui.deleteButton': 'deleteNode'
                },
                onRender: function () {
                    var self = this;

                    if (_.contains(['emotion', 'gesture'], this.model.get('name'))) {
                        // set default
                        if (!this.model.get('magnitude'))
                            this.model.set('magnitude', 1);

                        this.ui.magnitudeLabel.html(this.model.get('magnitude') * 100 + '%');

                        // init slider
                        this.ui.magnitudeSlider.slider({
                            animate: true,
                            range: 'min',
                            value: this.model.get('magnitude') * 100,
                            slide: function (e, ui) {
                                self.model.set('magnitude', (ui.value / 100).toFixed(2));
                                self.ui.magnitudeLabel.html(self.model.get('magnitude') * 100 + '%');

                            }
                        });
                    }

                    switch (this.model.get('name')) {
                        case 'emotion':
                            // load emotions
                            api.getAvailableEmotionStates(function (emotions) {
                                _.each(emotions, function (emotion) {
                                    $(self.ui.emotionSelect).append($('<option>').prop('value', emotion).html(emotion));
                                });

                                if (!self.model.get('emotion') && emotions.length > 0)
                                    self.model.set('emotion', emotions[0]);

                                if (self.model.get('emotion'))
                                    $(self.ui.emotionSelect).val(self.model.get('emotion'));

                                $(self.ui.emotionSelect).select2();
                            });
                            break;
                        case 'gesture':
                            // load gestures
                            api.getAvailableGestures(function (gestures) {
                                _.each(gestures, function (gesture) {
                                    $(self.ui.gestureSelect).append($('<option>').prop('value', gesture).html(gesture));
                                });

                                if (!self.model.get('gesture') && gestures.length > 0) {
                                    self.model.set('gesture', gestures[0]);
                                    self.setGestureLength();
                                }

                                if (self.model.get('gesture'))
                                    $(self.ui.emotionSelect).val(self.model.get('gesture'));

                                $(self.ui.gestureSelect).select2();
                            });

                            if (!this.model.get('speed')) this.model.set('speed', 1);

                            this.ui.speedLabel.html(this.model.get('speed'));
                            this.ui.speedSlider.slider({
                                range: 'min',
                                animate: true,
                                min: 50,
                                max: 200,
                                value: this.model.get('speed') * 100,
                                slide: function (e, ui) {
                                    var speed = ui.value / 100;
                                    self.model.set('speed', speed);
                                    self.ui.speedLabel.html(speed.toFixed(2));
                                }
                            });
                            break;
                        case 'look_at':
                            this.buildCrosshair();
                            break;
                        case 'speech':
                            if (this.model.get('text')) this.ui.textInput.val(this.model.get('text'));
                            break;
                    }
                },
                setDuration: function () {
                    this.model.set('duration', Number($(this.ui.duration).val()));
                },
                setText: function () {
                    this.model.set('text', this.ui.textInput.val());
                },
                setTextDuration: function () {
                    var self = this;
                    api.getTtsLength(this.ui.textInput.val(), function (response) {
                        self.model.set('duration', response.length);
                    });
                },
                setEmotion: function () {
                    this.model.set('emotion', this.ui.emotionSelect.val());
                },
                setGesture: function () {
                    this.model.set('gesture', this.ui.gestureSelect.val());
                    this.setGestureLength();
                },
                setGestureLength: function () {
                    var self = this;
                    api.getAnimationLength(this.model.get('gesture'), function (response) {
                        self.model.set('duration', response.length);
                    });
                },
                setStartTime: function () {
                    this.model.set('start_time', Number($(this.ui.startTime).val()));
                },
                buildCrosshair: function () {
                    var self = this;
                    $(this.ui.crosshair).crosshairsl($.extend({}, {
                        xmin: -1.5,
                        xmax: 1.5,
                        xval: this.model.get('x') ? this.model.get('x') : 0,
                        ymin: -1.5,
                        ymax: 1.5,
                        yval: this.model.get('y') ? this.model.get('y') : 0,
                        change: function (e, ui) {
                            self.model.set('x', ui.xval);
                            self.model.set('y', ui.yval);

                            self.model.call();
                        }
                    }, {
                        bgColor: "#485563",
                        fgColor: "#fff"
                    }));
                },
                deleteNode: function () {
                    var self = this;

                    this.model.destroy();
                    this.$el.slideUp(null, function () {
                        self.destroy();
                    });
                }
            });
        });
    });
