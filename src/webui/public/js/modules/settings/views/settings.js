define(['application', 'marionette', 'tpl!./templates/settings.tpl', 'json_editor'],
    function (App, Marionette, template, JSONEditor) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                settings: '.app-settings'
            },
            modelEvents: {
                change: 'setConfig'
            },
            onRender: function () {
                var self = this;
                this.editor = new JSONEditor(this.ui.settings.get(0), {
                    form_name_root: 'config',
                    theme: 'bootstrap3',
                    schema: this.options.schema,
                    disable_collapse: true,
                    disable_edit_json: true,
                    disable_array_reorder: true,
                    disable_properties: true,
                    iconlib: "fontawesome4"
                });

                this.editor.on('change',function() {
                    self.update();
                });

                this.setConfig();

                if (this.options.refresh)
                    this.refreshInterval = setInterval(function () {
                        console.log('refresh');
                        self.model.fetch()
                    }, 1000);
            },
            onDestroy: function () {
                clearInterval(this.refreshInterval);
            },
            setConfig: function () {
                if (this.editor)
                    this.editor.setValue(this.model.toJSON());
            },
            update: function () {
                if (this.editor.validate().length === 0) {
                    this.model.save(this.editor.getValue(), {
                        error: function () {
                            console.log('error updating node configuration');
                        }
                    });
                }
            }
        });
    });