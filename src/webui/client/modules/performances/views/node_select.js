define(['application', 'marionette', './templates/node_select.tpl', '../entities/node',
        '../../settings/views/settings', '../../settings/entities/node_config_schema', 'lib/api', 'underscore',
        'jquery', 'jquery-ui', 'lib/crosshair-slider', 'select2', 'select2-css'],
    function (App, Marionette, template, Node, SettingsView, SettingsSchemaModel, api, _, $) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                nodeProperties: '[data-node-property]',

                emotionList: '.app-emotion-list',
                gestureList: '.app-gesture-list',
                somaList: '.app-soma-list',
                expressionList: '.app-expression-list',
                kfAnimationList: '.app-kfanimation-list',
                textInput: '.app-node-text',
                langSelect: 'select.app-lang-select',
                attentionRegionList: '.app-attention-region-list',
                topicInput: '.app-node-topic',
                btreeModeSelect: 'select.app-btree-mode-select',
                speechEventSelect: 'select.app-speech-event-select',
                hrAngleSlider: '.app-hr-angle-slider',
                hrAngleLabel: '.app-hr-angle-label',
                listenResponseTemplate: '.app-listen-response-template',
                listenResponseInputs: '.app-listen-response-template input',
                listenAddResponseButton: '.app-listen-add-response',
                listenResponseList: '.app-chat-response-list',
                removeListenResponseButton: '.app-remove-listen-response',
                enableChatbotCheckbox: '.app-enable-chatbot-checkbox',
                responsesProperty: '[data-node-property="responses"]'
            },
            events: {
                'keyup @ui.textInput': 'setText',
                'change @ui.textInput': 'setTextDuration',
                'change @ui.langSelect': 'setLanguage',
                'change @ui.topicInput': 'setTopic',
                'change @ui.listenResponseInputs': 'updateChatResponses',
                'click @ui.listenAddResponseButton': 'addListenResponse',
                'click @ui.removeListenResponseButton': 'removeListenResponse',
                'change @ui.enableChatbotCheckbox': 'setEnableChatbot'
            },
            regions: {
                settingsEditor: '.app-settings-editor'
            },
            modelEvents: {
                change: 'modelChanged'
            },
            modelChanged: function () {
                this.ui.topicInput.val(this.model.get('topic'));
            },
            onAttach: function () {
                this.initFields();
            },
            initFields: function () {
                var self = this,
                    properties = this.model.getConfig().properties;

                // display node specific properties
                this.ui.nodeProperties.hide();
                _.each(properties, function (prop) {
                    self.ui.nodeProperties.filter('[data-node-property="' + prop + '"]').show();
                });

                this.modelChanged();

                if (this.model.hasProperty('enable_chatbot')) {
                    this.ui.enableChatbotCheckbox.prop('checked', !!this.model.get('enable_chatbot'));
                    this.setEnableChatbot();
                }

                if (this.model.hasProperty('responses')) {
                    this.initChatReponses();
                    this.ui.listenAddResponseButton.click();
                }

                if (this.model.hasProperty('emotion')) {
                    // init with empty list
                    self.updateEmotions([]);
                    // load emotions
                    api.getAvailableEmotionStates(function (emotions) {
                        self.updateEmotions(emotions);
                    });
                }

                if (this.model.hasProperty('expression')) {
                    // init with empty list
                    self.updateExpressions([]);
                    // load emotions
                    api.expressionList(function (expressions) {
                        self.updateExpressions(expressions.exprnames)
                    });
                }

                if (this.model.hasProperty('kfanimation')) {
                    this.model.on('change', this.setKFAnimationDuration, this);

                    // init with empty list
                    self.updateKFAnimations([]);
                    // load emotions
                    api.getAnimations(function (animations) {
                        animations = _.pluck(animations, 'name');
                        self.updateKFAnimations(animations)
                    });
                }

                if (this.model.hasProperty('angle')) {
                    if (!this.model.get('angle'))
                        this.model.set('angle', 0);

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
                }

                if (this.model.hasProperty('animation')) {
                    // init with empty list
                    self.updateGestures([]);
                    // load gestures
                    api.getAvailableGestures(function (gestures) {
                        self.updateGestures(gestures)
                    });

                    this.model.on('change', this.setGestureLength, this);
                }

                if (this.model.hasProperty('soma')) {
                    // init with empty list
                    self.updateSomaStates([]);
                    // load gestures
                    api.getAvailableSomaStates(function (somas) {
                        self.updateSomaStates(somas)
                    });
                }

                if (this.model.hasProperty('attention_region')) {
                    this.enableAttentionRegionSelect();
                }

                if (this.model.hasProperty('text')) {
                    if (!this.model.get('text'))
                        this.model.set('text', '');
                    this.ui.textInput.val(this.model.get('text'));
                }

                if (this.model.hasProperty('language')) {
                    if (!this.model.get('lang'))
                        this.model.set('lang', 'en');
                    this.ui.langSelect.val(this.model.get('lang'));
                    $(self.ui.langSelect).select2();
                }

                if (this.model.hasProperty('btree_mode')) {
                    if (!this.model.get('mode'))
                        this.model.set('mode', 255);
                    this.ui.btreeModeSelect.val(this.model.get('mode'));
                    $(self.ui.btreeModeSelect).select2();
                }
                if (this.model.hasProperty('rosnode')) {
                    this.setSettingsEditor(this.model.get('schema'));
                    this.listenTo(this.model, 'change:rosnode', function () {
                        self.updateSettingsSchema();
                    });
                }

                if (this.model.hasProperty('speech_event')) {
                    if (!this.model.get('chat'))
                        this.model.set('chat', '');
                    this.ui.speechEventSelect.val(this.model.get('chat'));
                    this.ui.speechEventSelect.select2();
                }

                if (this.model.hasProperty('message')) {
                    if (!this.model.get('message'))
                        this.model.set('message', '');
                    this.ui.messageInput.val(this.model.get('message'));
                }
            },
            initList: function (list, attr, container, options) {
                if (this.isDestroyed) return;
                var self = this;
                options = options || {};
                container.html('');
                _.each(list, function (label, val) {
                    if (list && list.constructor === Array) val = label;

                    var thumbnail = $('<div>').addClass('app-node-thumbnail')
                        .attr('data-node-name', self.model.get('name')).attr('data-' + attr, val)
                        .html($('<span>').html(label)).click(function () {
                            self.model.set(attr, val);
                            $('[data-' + attr + ']', container).removeClass('active');
                            $(this).addClass('active');
                            if (options.change) options.change(val);
                        }).draggable({
                            helper: function () {
                                var node = self.model;

                                if (self.collection && self.collection.contains(node)) {
                                    var attributes = node.toJSON();
                                    delete attributes['id'];
                                    node = Node.create(attributes);
                                }

                                node.set(attr, val);
                                return $('<span>').attr('data-node-name', node.get('name'))
                                    .attr('data-node-id', node.get('id'))
                                    .addClass('label app-node').html(node.getTitle());
                            },
                            appendTo: 'body',
                            revert: 'invalid',
                            delay: 100,
                            snap: '.app-timeline-nodes',
                            snapMode: 'inner',
                            zIndex: 1000,
                            cursor: 'move',
                            cursorAt: {top: 0, left: 0}
                        });

                    container.append(thumbnail);
                });

                var update = function () {
                    if (self.model.get(attr)) {
                        $('[data-' + attr + ']', container).removeClass('active');
                        $('[data-' + attr + '="' + self.model.get(attr) + '"]', container).addClass('active');
                    }
                };

                this.model.on('change:' + attr, update);
                update();
            },
            updateEmotions: function (emotions) {
                this.initList(emotions, 'emotion', this.ui.emotionList);
            },
            setKFAnimationDuration: function () {
                var self = this;
                api.getKFAnimationLength(this.model.get('animation'), function (response) {
                    self.animationFrames = response.frames;
                    self.model.set('duration', 0.1 + self.animationFrames / self.model.get('fps'));
                });
            },
            updateKFAnimations: function (animations) {
                this.initList(animations, 'animation', this.ui.kfAnimationList);
                $(this.ui.kfModeSelect).select2();
            },
            updateExpressions: function (expressions) {
                this.initList(expressions, 'expression', this.ui.expressionList);
            },
            updateGestures: function (gestures) {
                this.initList(gestures, 'gesture', this.ui.gestureList);
            },
            updateSomaStates: function (somas) {
                this.initList(somas, 'soma', this.ui.somaList);
            },
            updateSettingsSchema: function () {
                var self = this;
                var rosnode = this.model.get('rosnode');
                api.services.get_node_description.callService({node: rosnode}, function (response) {
                    var schema = SettingsSchemaModel.getSchemaFromDesc(JSON.parse(response.description), rosnode);
                    self.model.set({schema: schema, values: {}});
                    self.setSettingsEditor(schema);
                    api.services.get_node_configuration.callService({node: rosnode}, function (response) {
                        self.model.set({values: JSON.parse(response.configuration)});
                    }, function (error) {
                        console.log("Cant retrieve node settings")
                    });
                }, function (error) {
                    console.log('Error fetching configuration schema');
                });
            },
            setSettingsEditor: function (schema) {
                this.getRegion('settingsEditor').show(new SettingsView({
                    model: this.model,
                    schema: schema,
                    refresh: false
                }));
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
            setGestureLength: function () {
                var self = this;
                api.getAnimationLength(this.model.get('gesture'), function (response) {
                    self.gestureDuration = response.length;
                    self.model.set('duration', self.gestureDuration / self.model.get('speed'));
                });
            },
            setTopic: function () {
                this.model.set('topic', this.ui.topicInput.val());
            },
            enableAttentionRegionSelect: function () {
                var self = this;

                api.getRosParam('/' + api.config.robot + '/webui/attention_regions', function (regions) {
                    regions = regions || {};
                    _.each(regions, function (r, i) {
                        regions[i] = r['label'];
                    });
                    regions.custom = 'custom';
                    self.initList(regions, 'attention_region', self.ui.attentionRegionList);
                });
            },
            initChatReponses: function () {
                var self = this;
                self.ui.listenResponseList.html('');

                _.each(this.model.get('responses'), function (response) {
                    var template = self.ui.listenResponseTemplate.clone(),
                        input = $('.app-chat-input', template),
                        output = $('.app-chat-output', template);
                    input.val(response['input']);
                    output.val(response['output']);
                    self.ui.listenResponseList.append(template.hide().fadeIn());
                });
            },
            updateChatResponses: function () {
                var responses = [],
                    inputs = $('input', this.ui.listenResponseList),
                    i;

                for (i = 0; i < inputs.length / 2; i++) {
                    var input = $(inputs[i * 2]).val(),
                        output = $(inputs[i * 2 + 1]).val();

                    if (input && output) responses.push({input: input, output: output});
                }

                this.model.set('responses', responses);
            },
            addListenResponse: function () {
                this.ui.listenResponseList.append(this.ui.listenResponseTemplate.clone().hide().fadeIn());
            },
            removeListenResponse: function (e) {
                var self = this;
                $(e.target).closest('.app-listen-response-template').fadeOut(100, function () {
                    $(this).remove();
                    self.updateChatResponses();
                });
            },
            setEnableChatbot: function () {
                var checked = this.ui.enableChatbotCheckbox.is(':checked');

                this.model.set('enable_chatbot', checked ? '1' : '');

                if (checked)
                    this.ui.responsesProperty.fadeOut();
                else
                    this.ui.responsesProperty.fadeIn();
            }
        });
    });
