define(['application', 'marionette', 'tpl!./templates/node_settings.tpl', 'lib/api', 'underscore', 'jquery-ui',
        'lib/crosshair-slider', 'select2'],
    function (App, Marionette, template, api, _) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                nodeProperties: '[data-node-property]',
                magnitudeSlider: '.app-magnitide-slider',
                emotionList: '.app-emotion-list',
                gestureList: '.app-gesture-list',
                somaList: '.app-soma-list',
                expressionList: '.app-expression-list',
                textInput: '.app-node-text',
                startTime: '.app-node-start-time',
                duration: '.app-node-duration',
                crosshair: '.app-crosshair',
                speedSlider: '.app-speed-slider',
                speedLabel: '.app-speed-label',
                magnitudeLabel: '.app-magnitude-label',
                langSelect: 'select.app-lang-select',
                topicInput: '.app-node-topic',
                fpsSlider: '.app-fps-slider',
                fpsLabel: '.app-fps-label',
                kfAnimationList: '.app-kfanimation-list',
                messageInput: '.app-node-message-input',
                kfModeSelect: 'select.app-kfmode-select',
                btreeModeSelect: 'select.app-btree-mode-select',
                speechEventSelect: 'select.app-speech-event-select',
                hrAngleSlider: '.app-hr-angle-slider',
                hrAngleLabel: '.app-hr-angle-label',
                attentionRegionList: '.app-attention-region-list',
                timeout: '.app-node-timeout'
            },
            events: {
                'change @ui.duration': 'setDuration',
                'change @ui.startTime': 'setStartTime',
                'keyup @ui.textInput': 'setText',
                'change @ui.textInput': 'setTextDuration',
                'change @ui.langSelect': 'setLanguage',
                'change @ui.topicInput': 'setTopic',
                'change @ui.messageInput': 'setMessage',
                'change @ui.btreeModeSelect': 'setBtreeMode',
                'change @ui.speechEventSelect': 'setSpeechEvent',
                'change @ui.kfModeSelect': 'setKFMode',
                'change @ui.timeout': 'updateTimeout'
            },
            modelEvents: {
                change: 'modelChanged'
            },
            modelChanged: function () {
                this.ui.startTime.val(this.model.get('start_time'));
                this.ui.duration.val(this.model.get('duration'));
                this.ui.topicInput.val(this.model.get('topic'));
            },
            onRender: function () {
                this.initFields();
            },
            initFields: function () {
                var self = this,
                    properties = this.model.getConfig().properties;

                this.ui.nodeProperties.hide();
                if (properties)
                    _.each(properties, function (prop) {
                        self.ui.nodeProperties.filter('[data-node-property="' + prop + '"]').show();
                    });

                this.modelChanged();

                if (_.contains(['pause'], this.model.get('name'))) {
                    this.ui.timeout.val(this.model.get('timeout') || '');
                }

                if (_.contains(['emotion', 'gesture', 'expression'], this.model.get('name'))) {
                    var magnitude = this.model.get('magnitude') || 0;

                    // set default
                    if (!this.model.get('magnitude'))
                        this.model.set('magnitude', [0.9, 1]);

                    if (magnitude instanceof Array && magnitude.length == 2)
                        magnitude = [magnitude[0] * 100, magnitude[1] * 100];
                    else // computable with previous version of single value magnitude
                        magnitude = [magnitude * 100, magnitude * 100];

                    this.ui.magnitudeLabel.html(magnitude[0] + '-' + magnitude[1] + '%');

                    // init slider
                    this.ui.magnitudeSlider.slider({
                        animate: true,
                        range: true,
                        values: magnitude,
                        slide: function (event, ui) {
                            self.model.set('magnitude', [ui.values[0] / 100, ui.values[1] / 100]);
                            self.ui.magnitudeLabel.html(ui.values[0] + '-' + ui.values[1] + '%');
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
                        api.expressionList(function (expressions) {
                            self.updateExpressions(expressions.exprnames)
                        });
                        break;
                    case 'kfanimation':
                        // init with empty list
                        self.updateKFAnimations([]);
                        // load emotions
                        api.getAnimations(function (animations) {
                            self.updateKFAnimations(animations)
                        });
                        // init slider
                        if (!this.model.get('fps')) this.model.set('fps', 24);
                        self.ui.fpsLabel.html(Math.floor(self.model.get('fps')) + ' fps');
                        // Disable blender head output by default
                        if (!this.model.get('blender_mode')) this.model.set('blender_mode', 'no');
                        self.ui.kfModeSelect.val(this.model.get('blender_mode'));

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
                    case 'head_rotation':
                        if (!this.model.get('angle')) this.model.set('angle', 0);
                        this.ui.hrAngleSlider.slider({
                            animate: true,
                            range: 'min',
                            min: -50,
                            max: 50,
                            value: this.model.get('angle') * 100,
                            slide: function (e, ui) {
                                self.model.set('angle', 0 - parseFloat(ui.value) / 100.0);
                                self.model.call();
                                self.ui.hrAngleLabel.html(parseFloat(self.model.get('angle')).toFixed(2) + ' rad');
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
                    case 'soma':
                        // init with empty list
                        self.updateSomaStates([]);
                        // load gestures
                        api.getAvailableSomaStates(function (somas) {
                            self.updateSomaStates(somas)
                        });
                        break;
                    case 'look_at':
                        this.enableAttentionRegionSelect();
                        break;
                    case 'gaze_at':
                        this.enableAttentionRegionSelect();
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
                    case 'interaction':
                        if (!this.model.get('mode'))
                            this.model.set('mode', 255);
                        this.ui.btreeModeSelect.val(this.model.get('mode'));
                        $(self.ui.btreeModeSelect).select2();
                        if (!this.model.get('chat'))
                            this.model.set('chat', '');
                        this.ui.speechEventSelect.val(this.model.get('chat'));
                        $(self.ui.speechEventSelect).select2();
                        break;
                    case 'pause':
                        this.model.set('duration', 0.2);
                        break;
                    case 'chat_pause':
                        if (!this.model.get('message'))
                            this.model.set('message', '');
                        this.ui.messageInput.val(this.model.get('message'));
                }
            },
            updateList: function (list, attr, container, options) {
                if (this.isDestroyed) {
                    return;
                }
                var self = this;
                options = options || {};
                container.html('');
                _.each(list, function (val, key) {
                    var name = key;
                    if (list && list.constructor === Array) name = val;

                    container.append($('<div>').addClass('app-node-thumbnail').attr('data-' + attr, name)
                        .html($('<span>').html(val)).click(function () {
                            self.model.set(attr, name);
                            $('[data-' + attr + ']', container).removeClass('active');
                            $(this).addClass('active');
                            if (options.change) options.change(name);
                        }));
                });

                if (this.model.get(attr))
                    $('[data-' + attr + '="' + this.model.get(attr) + '"]', container).addClass('active');
            },
            updateEmotions: function (emotions) {
                this.updateList(emotions, 'emotion', this.ui.emotionList);
            },
            updateKFAnimations: function (animations) {
                this.updateList(animations, 'animation', this.ui.kfAnimationList);
                $(this.ui.kfModeSelect).select2();
            },
            updateExpressions: function (expressions) {
                this.updateList(expressions, 'expression', this.ui.expressionList);
            },
            updateGestures: function (gestures) {
                this.updateList(gestures, 'gesture', this.ui.gestureList);
            },
            updateSomaStates: function (somas) {
                this.updateList(somas, 'soma', this.ui.somaList);
            },
            setDuration: function () {
                this.model.set('duration', Number($(this.ui.duration).val()));
            },
            setMessage: function () {
                this.model.set('message', this.ui.messageInput.val());
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
                });
            },
            setEmotion: function () {
                this.model.set('emotion', this.ui.emotionList.val());
            },
            setExpression: function () {
                this.model.set('expression', this.ui.expressionList.val());
            },
            setGesture: function () {
                this.model.set('gesture', this.ui.gestureList.val());
                this.setGestureLength();
            },
            setSoma: function () {
                this.model.set('soma', this.ui.somaSelect.val());
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
            setKFMode: function () {
                this.model.set('blender_mode', $(this.ui.kfModeSelect).val());
            },
            setTopic: function () {
                this.model.set('topic', this.ui.topicInput.val());
            },
            setKFAnimation: function () {
                this.model.set('animation', this.ui.kfAnimationList.val())
                this.setKFAnimationDuration();
            },
            setKFAnimationDuration: function () {
                var self = this;
                api.getKFAnimationLength(this.model.get('animation'), function (response) {
                    self.animationFrames = response.frames;
                    self.model.set('duration', 0.1 + self.animationFrames / self.model.get('fps'));
                });
            },
            setBtreeMode: function () {
                this.model.set('mode', parseInt(this.ui.btreeModeSelect.val()));
            },
            setSpeechEvent: function () {
                this.model.set('chat', this.ui.speechEventSelect.val());
            },
            enableAttentionRegionSelect: function () {
                var self = this;

                this.buildCrosshair();
                this.ui.crosshair.hide();

                api.getRosParam('/' + api.config.robot + '/webui/attention_regions', function (regions) {
                    regions = regions || {};
                    regions.custom = 'custom';
                    self.updateList(regions, 'attention_region', self.ui.attentionRegionList, {
                        change: function () {
                            self.selectAttentionRegion();
                        }
                    });
                    self.selectAttentionRegion();
                });
            },
            buildCrosshair: function () {
                var self = this;
                $(this.ui.crosshair).crosshairsl({
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
                });

                self.model.set('x', 1);
                self.model.set('y', 0);
                self.model.set('z', 0);
            },
            selectAttentionRegion: function () {
                if (!this.isDestroyed) {
                    if (this.model.get('attention_region') == 'custom')
                        this.ui.crosshair.fadeIn();
                    else
                        this.ui.crosshair.fadeOut();
                }
            },
            updateTimeout: function () {
                this.model.set('timeout', this.ui.timeout.val());
            }
        });
    });
