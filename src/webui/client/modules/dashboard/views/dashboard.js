define(["marionette", "./templates/dashboard.tpl", 'jquery', 'lib/api', 'vendor/jquery.knob', './css/dashboard'],
    function (Marionette, template, $, api) {
        var self;
        return Marionette.LayoutView.extend({
            template: template,
            activeExpression: 'neutral',
            intensity: 0.5,
            ui: {
                intensity: '.app-intensity',
                performanceButtons: '.app-performance',
                expressionButtons: '.app-expression',
                sayButton: '.app-say-button',
                clearButton: '.app-clear-button',
                speechInput: '.app-say-textarea'
            },
            events: {
                'click @ui.performanceButtons': 'runPerformance',
                'click @ui.expressionButtons': 'playExpression',
                'click @ui.sayButton': 'say',
                'click @ui.clearButton': 'clearVal',
                'click @ui.recordButton': 'toggleSpeech'
            },
            serializeData: function () {
                return this.options;
            },
            initialize: function (options) {
                self = this;
                if (options.expressions)
                    options.expressions.bind('change add remove reset', this.render, this);
            },
            onDestroy: function () {
                this.options.expressions.unbind('change add remove reset', this.render, this);
            },
            config: {
                knob: {
                    max: 33,
                    maxShow: 11,
                    fgColor: "#df691a",
                    fgColorExtreme: "#df691a"
                }
            },
            onRender: function () {
                $(this.ui.intensity).knob({
                    min: 0, max: self.config.knob.max,
                    angleOffset: -125, angleArc: 275,
                    width: 200, height: 200,
                    thickness: 0.4,
                    fgColor: '#df691a', bgColor: "#444",
                    draw: function () { //Extended the draw function to add a ring around the dial.
                        this.g.lineWidth = 3;
                        this.cursorExt = 0.2;

                        //Override radius to fit the outer circle inside canvas
                        this.radius = this.xy - this.lineWidth * 2 / 3 - this.g.lineWidth;

                        this.g.beginPath();
                        this.g.strokeStyle = this.o.fgColor;
                        this.g.arc(this.xy, this.xy, this.radius + 1 + this.lineWidth * 2 / 3, 0, 2 * Math.PI, false);
                        this.g.stroke();
                        return true;
                    },
                    change: function (value) {
                        self.updateIntensity(value);
                    },
                    format: function (value) {
                        return Math.round(value / self.config.knob.max * self.config.knob.maxShow);
                    }
                });
                self.blenderEnable();
                api.enableInteractionMode();

            },
            runPerformance: function (e) {
                self.blenderEnable();
                var performance = self.options.performances.findWhere({name: $(e.target).data('performance')});
                if (performance) performance.run();
            },
            playExpression: function (e) {
                this.ui.expressionButtons.removeClass('active');
                $(e.target).addClass('active');
                self.blenderDisable();
                self.activeExpression = $(e.target).data('expression');
                api.setExpression(self.activeExpression, self.intensity);
            },
            say: function () {
                self.blenderEnable();
                api.robotSpeech(this.ui.speechInput.val());
            },
            clearVal: function(){
                this.ui.speechInput.val("");
            },
            toggleSpeech: function () {
                if (this.speechEnabled)
                    this.stopSpeech();
                else
                    this.startSpeech();
            },
            startSpeech: function () {
                if (!this.speechRecognition) {
                    var mostConfidentResult = null;

                    if ('webkitSpeechRecognition' in window) {
                        this.speechRecognition = new webkitSpeechRecognition();
                    } else if ('SpeechRecognition' in window) {
                        this.speechRecognition = new SpeechRecognition();
                    } else {
                        console.log('webspeech api not supported');
                        this.speechRecognition = null;
                    }

                    this.speechRecognition.interimResults = true;
                    this.speechRecognition.continuous = true;

                    this.speechRecognition.onstart = function () {
                        console.log('starting webspeech');
                        self.speechEnabled = true;
                        self.ui.recordButton.removeClass('btn-info').addClass('btn-danger');
                    };

                    this.speechRecognition.onresult = function (event) {
                        _.each(event.results, function (results) {
                            _.each(results, function (result) {
                                if (!mostConfidentResult || mostConfidentResult.confidence <= result.confidence)
                                    mostConfidentResult = result;
                            });
                        });
                    };

                    this.speechRecognition.onerror = function (event) {
                        console.log('error recognising speech');
                        console.log(event);
                    };
                    this.speechRecognition.onend = function () {
                        self.speechEnabled = false;
                        self.ui.recordButton.addClass('btn-info').removeClass('btn-danger');

                        if (mostConfidentResult && mostConfidentResult.confidence > 0.5) {
                            console.log('speech recognised: ' + mostConfidentResult.transcript);
                            api.robotSpeech(mostConfidentResult.transcript);
                        }

                        mostConfidentResult = null;
                    };
                }

                if (this.speechRecognition && !this.speechEnabled)
                    this.speechRecognition.start();

                return this.speechRecognition;
            },
            stopSpeech: function () {
                if (this.speechRecognition && this.speechEnabled)
                    this.speechRecognition.stop();
            },
            updateIntensity: function (value) {
                if (!value)
                    value = parseInt(self.ui.intensity.val()) / self.config.knob.maxShow * self.config.knob.max;

                self.intensity = (Math.round(value / self.config.knob.max *
                    self.config.knob.maxShow) < self.config.knob.maxShow)
                    ? value / (self.config.knob.max * 1.2) : 1.0;
                self.blenderDisable();
                api.setExpression(self.activeExpression, self.intensity);
            },
            blenderEnable: function(){
                api.blenderMode.enable();
                clearInterval(self.blender_timer);
            },
            blenderDisable: function(){
               api.blenderMode.disableFace();
               clearInterval(self.blender_timer);
               self.blender_timer = setInterval(function(){self.blenderEnable()}, 5000);
            }
        });
    });
