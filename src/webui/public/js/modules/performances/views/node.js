define(['application','marionette',  'tpl!./templates/node.tpl', 'lib/api', 'bootbox', 'jquery-ui', 'lib/crosshair-slider',
        'select2'],
    function (App, Marionette, template, api, bootbox) {
        return Marionette.ItemView.extend({
                template: template,
                ui: {
                    magnitudeSlider: '.app-magnitide-slider',
                    emotionSelect: 'select.app-emotion-select',
                    gestureSelect: 'select.app-gesture-select',
                    expressionSelect: 'select.app-expression-select',
                    textInput: '.app-node-text',
                    startTime: '.app-node-start-time',
                    duration: '.app-node-duration',
                    crosshair: '.app-crosshair',
                    deleteButton: '.app-delete-node-button',
                    speedSlider: '.app-speed-slider',
                    speedLabel: '.app-speed-label',
                    magnitudeLabel: '.app-magnitude-label',
                    langSelect: 'select.app-lang-select',
                    frameCount: '.app-node-frames-indicator',
                    durationIndicator: '.app-node-duration-indicator',
                    topicInput: '.app-node-topic',
                    fpsSlider: '.app-fps-slider',
                    fpsLabel: '.app-fps-label',
                    kfAnimationSelect: 'select.app-kfanimation-select'
                },
                events: {
                    'change @ui.duration': 'setDuration',
                    'change @ui.startTime': 'setStartTime',
                    'keyup @ui.textInput': 'setText',
                    'change @ui.textInput': 'setTextDuration',
                    'change @ui.langSelect': 'setLanguage',
                    'change @ui.emotionSelect': 'setEmotion',
                    'change @ui.gestureSelect': 'setGesture',
                    'change @ui.expressionSelect': 'setExpression',
                    'change @ui.topicInput': 'setTopic',
                    'change @ui.kfAnimationSelect': 'setKFAnimation',
                    'click @ui.deleteButton': 'deleteNode'
                },
                onRender: function () {
                    var self = this;

                if (_.contains(['pause'], this.model.get('name'))) {
                    this.ui.durationIndicator.hide();
                    this.ui.frameCount.hide();
                } else
                    this.updateIndicators();

                if (_.contains(['emotion', 'gesture', 'expression'], this.model.get('name'))) {
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
                            self.ui.magnitudeLabel.html(Math.floor(self.model.get('magnitude') * 100) + '%');
                        }
                    });
                }

                    switch (this.model.get('name')) {
                        case 'emotion':
                            // init with empty list
                            self.updateEmotions([]);
                            // load emotions
                            api.getAvailableEmotionStates(function (emotions) {
                                self.updateEmotions(emotions);
                            });
                            break;
                        case 'expression':
                            // init with empty list
                            self.updateExpressions([]);
                            // load emotions
                            api.expressionList(function(expressions) { self.updateExpressions(expressions.exprnames) });
                            break;
                        case 'kfanimation':
                            // init with empty list
                            self.updateKFAnimations([]);
                            // load emotions
                            api.getAnimations(function(animations) { self.updateKFAnimations(animations) });
                            // init slider
                            if (!this.model.get('fps')) this.model.set('fps', 24);
                            self.ui.fpsLabel.html(Math.floor(self.model.get('fps')) + ' fps');
                            this.ui.fpsSlider.slider({
                                animate: true,
                                range: 'min',
                                min: 12,
                                max: 48,
                                value: this.model.get('fps'),
                                slide: function (e, ui) {
                                    self.model.set('fps', ui.value);
                                    self.setKFAnimationDuration();
                                    self.ui.fpsLabel.html(Math.floor(self.model.get('fps')) + ' fps');
                                }
                            });
                            break;
                        case 'gesture':
                            // init with empty list
                            self.updateGestures([]);
                            // load gestures
                            api.getAvailableGestures(function (gestures) {
                                self.updateGestures(gestures)
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
                                self.setGestureLength();
                                self.ui.speedLabel.html(speed.toFixed(2));
                            }
                        });
                        break;
                    case 'look_at':
                        this.buildCrosshair();
                        break;
                    case 'gaze_at':
                        this.buildCrosshair();
                        break;
                    case 'speech':
                        if (this.model.get('text'))
                            this.ui.textInput.val(this.model.get('text'));
                        else
                            this.model.set('text', '');

                        if (!this.model.get('lang'))
                            this.model.set('lang', 'en');
                        this.ui.langSelect.val(this.model.get('lang'));
                        $(self.ui.langSelect).select2();
                        break;
                }
            },
            updateEmotions: function (emotions) {
                var self = this;
                _.each(emotions, function (emotion) {
                    $(self.ui.emotionSelect).append($('<option>').prop('value', emotion).html(emotion));
                });

                if (!this.model.get('emotion') && emotions.length > 0)
                    this.model.set('emotion', emotions[0]);

                if (this.model.get('emotion'))
                    $(this.ui.emotionSelect).val(this.model.get('emotion'));

                    $(this.ui.emotionSelect).select2();
                },
                updateKFAnimations: function (animations) {
                    var self = this;
                    _.each(animations, function (animation) {
                        $(self.ui.kfAnimationSelect).append($('<option>').prop('value', animation).html(animation));
                    });

                    if (!this.model.get('animation') && animations.length > 0){
                        this.model.set('animation', animations[0]);
                        this.setKFAnimationDuration();
                    }


                    if (this.model.get('animation'))
                        $(this.ui.kfAnimationSelect).val(this.model.get('animation'));

                    $(this.ui.kfAnimationSelect).select2();
                },
                updateExpressions: function (expressions) {
                    var self = this;
                    _.each(expressions, function (expr) {
                        $(self.ui.expressionSelect).append($('<option>').prop('value', expr).html(expr));
                    });

                if (!self.model.get('expression') && expressions.length > 0)
                    self.model.set('expression', expressions[0]);

                if (self.model.get('expression'))
                    $(self.ui.expressionSelect).val(self.model.get('expression'));

                $(self.ui.expressionSelect).select2();
            },
            updateGestures: function (gestures) {
                var self = this;
                _.each(gestures, function (gesture) {
                    $(self.ui.gestureSelect).append($('<option>').prop('value', gesture).html(gesture));
                });

                if (!this.model.get('gesture') && gestures.length > 0) {
                    this.model.set('gesture', gestures[0]);
                    this.setGestureLength();
                }

                if (this.model.get('gesture'))
                    $(this.ui.emotionSelect).val(this.model.get('gesture'));

                $(this.ui.gestureSelect).select2();
            },
            setDuration: function () {
                this.model.set('duration', $(this.ui.duration).val());
                this.updateIndicators();
            },
            updateIndicators: function () {
                var fps = App.getOption('fps'),
                    step = 1 / fps,
                    duration = Number(parseInt(this.model.get('duration') / step) * step).toFixed(2);

                    this.ui.durationIndicator.html(duration + 's');
                    this.ui.frameCount.html(parseInt(duration * fps));
                },
                setText: function () {
                    this.model.set('text', this.ui.textInput.val());
                },
                setLanguage: function () {
                    this.model.set('lang', this.ui.langSelect.val());
                },
                setTextDuration: function () {
                    var self = this;
                    api.getTtsLength(this.ui.textInput.val(), this.model.get('lang'), function (response) {
                        self.model.set('duration', response.length);
                        self.updateIndicators();
                    });
                },
                setEmotion: function () {
                    this.model.set('emotion', this.ui.emotionSelect.val());
                },
                setExpression: function () {
                    this.model.set('expression', this.ui.expressionSelect.val());
                },
                setGesture: function () {
                    this.model.set('gesture', this.ui.gestureSelect.val());
                    this.setGestureLength();
                },
                setGestureLength: function () {
                    var self = this;
                    api.getAnimationLength(this.model.get('gesture'), function (response) {
                        self.gestureDuration = response.length;
                        self.model.set('duration', self.gestureDuration / self.model.get('speed'));
                    });
                },
                setStartTime: function () {
                    this.model.set('start_time', Number($(this.ui.startTime).val()));
                },
                setTopic: function () {
                    this.model.set('topic', this.ui.topicInput.val());
                },
                setKFAnimation: function(){
                    this.model.set('animation', this.ui.kfAnimationSelect.val())
                    this.setKFAnimationDuration();
                },
                setKFAnimationDuration: function(){
                    var self = this;
                    api.getKFAnimationLength(this.model.get('animation'), function (response) {
                        self.animationFrames = response.frames;
                        self.model.set('duration', 0.1 + self.animationFrames / self.model.get('fps'));
                    });
                },
                buildCrosshair: function (params) {
                    var self = this;
                    params = params || {};
                    $(this.ui.crosshair).crosshairsl($.extend({}, {
                        xmin: -1,
                        xmax: 1,
                        xval: this.model.get('y') ? this.model.get('y') : 0,
                        ymin: -1,
                        ymax: 1,
                        yval: this.model.get('z') ? -1 * this.model.get('z') : 0,
                        change: function (e, ui) {
                            self.model.set('x', 1);
                            self.model.set('y', ui.xval);
                            self.model.set('z', -1 * ui.yval);

                            self.model.call();
                        }
                    }, params));

                self.model.set('x', 1);
                self.model.set('y', 0);
                self.model.set('z', 0);
            },
            deleteNode: function () {
                var self = this;

                bootbox.confirm("Are you sure?", function (result) {
                    if (result) {
                        self.model.destroy();
                        self.$el.slideUp(null, function () {
                            self.destroy();
                        });
                    }
                });
            }
        });
    });
