define(['application', 'marionette', './templates/settings.tpl', 'lib/regions/fade_in', './attention_regions', 'lib/api'
        , 'path', '../entities/settings'],
    function (App, Marionette, template, FadeInRegion, AttentionRegionsView, api, path, Settings) {
        return Marionette.View.extend({
            template: template,
            ui: {
                pauseBehaviorToggle: '.app-pause-behavior-toggle',
                keywords: '.app-keywords',
                tabs: '.app-tabs a',
                settingsTab: '.app-settings-tab',
                attentionTab: '.app-attention-tab',
                variableTemplate: '.app-variable-template',
                variableContainer: '.app-variables-container',
                addVariableButton: '.app-add-variable-btn',
                saveButton: '.app-save-btn',
                removeVariableButton: '.app-remove-variable-btn'
            },
            regions: {
                selectAreas: {
                    el: '.app-select-areas-content',
                    regionClass: FadeInRegion
                }
            },
            events: {
                'click @ui.addVariableButton': 'addVariable',
                'click @ui.pauseBehaviorToggle': 'togglePauseBehavior',
                'click @ui.removeVariableButton': 'removeVariable',
                'click @ui.saveButton': 'save'
            },
            initialize: function (options) {
                this.mergeOptions(options, ['path']);
                if (!this.model) this.model = new Settings({path: this.path});
            },
            onRender: function () {
                let self = this;
                this.model.fetch({
                    success: function () {
                        self.showKeywords();
                        self.showVariables();
                        self.showPauseBehaviorSwitch();
                    }
                });

                this.ui.tabs.on('shown.bs.tab', function (e) {
                    if ($(e.target).is(self.ui.attentionTab))
                        self.getRegion('selectAreas').show(new AttentionRegionsView({path: self.path}));
                });

                this.ui.settingsTab.tab('show');
            },
            save: function () {
                let self = this;

                this.setVariables();
                this.setKeywords();

                this.model.save({}, {
                    success: function () {
                        console.log('success');
                        App.Utilities.showPopover(self.ui.saveButton, 'Saved', 'right');
                    },
                    error: function () {
                        App.Utilities.showPopover(self.ui.saveButton, 'Unable to save', 'right');
                    }
                })
            },
            showVariables: function () {
                let self = this;
                $.each(this.model.get('variables') || {}, function (key, value) {
                    self.addVariable(key, value);
                });
            },
            showKeywords: function () {
                this.ui.keywords.val((this.model.get('keywords') || []).join(', '));
            },
            showPauseBehaviorSwitch: function () {
                let pauseBehavior = this.model.get('pause_behavior');
                if ((typeof pauseBehavior == 'boolean' && pauseBehavior) || typeof pauseBehavior != 'boolean')
                    this.ui.pauseBehaviorToggle.addClass('active');
                else
                    this.ui.pauseBehaviorToggle.removeClass('active').blur();
            },
            setKeywords: function () {
                let keywords = _.map(this.ui.keywords.val().split(','), function (k) {
                    return k.trim();
                });

                this.model.set('keywords', keywords);
            },
            setVariables: function () {
                let variables = {},
                    inputs = $('input', this.ui.variableContainer);

                for (let i = 0; i < inputs.length / 2; i++) {
                    let key = $(inputs[i * 2]).val(),
                        val = $(inputs[i * 2 + 1]).val();

                    if (key && val) variables[key] = val;
                }

                this.model.set('variables', variables);
            },
            togglePauseBehavior: function () {
                this.model.set('pause_behavior', !this.ui.pauseBehaviorToggle.hasClass('active'));
                this.showPauseBehaviorSwitch();
                
            },
            addVariable: function (key, value) {
                let field = this.ui.variableTemplate.clone().find('.form-group');
                if (key && value) {
                    field.find('.app-key-input').val(key);
                    field.find('.app-value-input').val(value);
                }

                this.ui.variableContainer.append(field);
            },
            removeVariable: function (e) {
                $(e.target).closest('.form-group').fadeOut(100, function () {
                    $(this).remove();
                });
            }
        });
    });
