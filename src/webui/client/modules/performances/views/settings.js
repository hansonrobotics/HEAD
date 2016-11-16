define(['application', 'marionette', './templates/settings.tpl', 'lib/regions/fade_in', './attention_regions', 'lib/api', 'path'],
    function (App, Marionette, template, FadeInRegion, AttentionRegionsView, api, path) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                keywords: '.app-keywords',
                saveKeywordsButton: '.app-save-keywords',
                tabs: '.app-tabs a',
                settingsTab: '.app-settings-tab',
                attentionTab: '.app-attention-tab',
                variableTemplate: '.app-variable-template',
                variableContainer: '.app-variables-container',
                addVariableButton: '.app-add-variable-btn',
                saveVariablesButton: '.app-save-variables-btn',
                removeVariableButton: '.app-remove-variable-btn'
            },
            regions: {
                selectAreas: {
                    el: '.app-select-areas-content',
                    regionClass: FadeInRegion
                }
            },
            events: {
                'click @ui.saveKeywordsButton': 'saveKeywords',
                'click @ui.addVariableButton': 'addVariable',
                'click @ui.removeVariableButton': 'removeVariable',
                'click @ui.saveVariablesButton': 'updateVariables'
            },
            initialize: function (options) {
                this.performancePath = options.performancePath || '.';
            },
            onRender: function () {
                var self = this;
                self.fetchKeywords();
                self.fetchVariables();
                this.ui.tabs.on('shown.bs.tab', function (e) {
                    if ($(e.target).is(self.ui.attentionTab))
                        self.getRegion('selectAreas').show(new AttentionRegionsView({performancePath: self.performancePath}));
                });
                this.ui.settingsTab.tab('show');
            },
            fetchKeywords: function () {
                var self = this;
                $.ajax({
                    method: 'GET',
                    url: path.join('/keywords', this.performancePath),
                    success: function (data) {
                        self.ui.keywords.val(data['keywords'].join(', '));
                    }
                });
            },
            saveKeywords: function () {
                var self = this;
                $.ajax({
                    method: 'POST',
                    url: path.join('/keywords', this.performancePath),
                    data: JSON.stringify({
                        path: this.performancePath,
                        keywords: this.ui.keywords.val().split(',')
                    }),
                    success: function () {
                        App.Utilities.showPopover(self.ui.saveKeywordsButton, 'Saved', 'right');
                    },
                    error: function () {
                        App.Utilities.showPopover(self.ui.saveKeywordsButton, 'Error', 'right');
                    }
                });
            },
            fetchVariables: function () {
                var self = this;
                $.ajax({
                    method: 'GET',
                    url: path.join('/performance/variables', this.performancePath),
                    success: function (data) {
                        $.each(data, function (key, value) {
                            self.addVariable(key, value);
                        });
                    },
                    error: function () {
                        console.log('Error fetching performance variables');
                    }
                });
            },
            updateVariables: function () {
                var self = this,
                    variables = {},
                    inputs = $('input', this.ui.variableContainer);

                for (var i = 0; i < inputs.length / 2; i++) {
                    var key = $(inputs[i * 2]).val(),
                        val = $(inputs[i * 2 + 1]).val();

                    if (key && val) variables[key] = val;
                }

                $.ajax({
                    method: 'POST',
                    url: path.join('/performance/variables', this.performancePath),
                    data: JSON.stringify(variables),
                    success: function () {
                        App.Utilities.showPopover(self.ui.saveVariablesButton, 'Saved', 'right');
                    },
                    error: function () {
                        App.Utilities.showPopover(self.ui.saveVariablesButton, 'Error', 'right');
                    }
                });
            },
            addVariable: function (key, value) {
                var field = this.ui.variableTemplate.clone().find('.form-group');
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
