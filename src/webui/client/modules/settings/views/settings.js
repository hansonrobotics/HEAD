define(['application', 'marionette', './templates/settings.tpl', 'json-editor', 'lib/json_editor/slider'],
    function (App, Marionette, template, JSONEditor) {
        return Marionette.View.extend({
            template: template,
            ui: {
                settings: '.app-settings'
            },
            modelEvents: {
                change: 'setConfig'
            },
            initialize: function (options) {
                this.mergeOptions(options, ['schema']);
            },
            onRender: function () {
                var self = this;
                // disable select2
                JSONEditor.defaults.editors.select.prototype.setupSelect2 = function() { this.select2 = null; };
                this.editor = new JSONEditor(this.ui.settings.get(0), {
                    form_name_root: 'config',
                    theme: 'bootstrap3',
                    schema: this.schema,
                    disable_collapse: true,
                    disable_edit_json: true,
                    disable_array_reorder: true,
                    disable_properties: true,
                    iconlib: 'fontawesome4'
                });

                this.editor.on('change', function () {
                    self.update();
                });

                this.setConfig();

                if (this.options.refresh)
                    this.refreshInterval = setInterval(function () {
                        self.model.fetch()
                    }, 1000);
            },
            onDestroy: function () {
                clearInterval(this.refreshInterval);
                this.editor.destroy();
            },
            setConfig: function () {
                var self = this,
                    values = this.model.toJSON();

                values = _.mapValues(values, function (val, key) {
                    if (values['node_schema'] && self.schema['properties'][key] && _.contains(['array', 'object'], self.schema['properties'][key]['type']))
                        return JSON.parse(val);
                    return val;
                });

                delete values['node_schema'];
                this.editor.setValue(values);
            },
            update: function () {
                var self = this;
                // Only valid settings could be saved in timeline
                if (this.editor.validate().length === 0) {
                    var values = this.editor.getValue();

                    values = _.mapValues(values, function (val) {
                        if (val && self.model.get('node_schema') && _.includes([Array, Object], val.constructor))
                            return JSON.stringify(val);
                        else
                            return val;
                    });

                    this.model.save(values, {
                        error: function () {
                            console.log('error updating node configuration');
                        }
                    });
                }
            }
        });
    });
