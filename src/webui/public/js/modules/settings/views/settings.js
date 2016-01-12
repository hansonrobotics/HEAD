define(['application', 'marionette', 'tpl!./templates/settings.tpl', 'json_editor'],
    function (App, Marionette, template, JSONEditor) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                settings: '.app-settings',
                update: '.app-update'
            },
            events: {
                'click @ui.update': 'update'
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
                    disable_properties: true,
                    iconlib: "bootstrap3"
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
            update: function (e) {
                if (this.editor.validate().length === 0) {
                    this.model.save(this.editor.getValue(), {
                        success: function (model) {
                            if (model.get('error')) {
                                model.unset('error');
                                App.Utilities.showPopover(e.target, 'Unable to save configuration')
                            } else
                                App.Utilities.showPopover(e.target, 'Success')
                        },
                        error: function () {
                            App.Utilities.showPopover(e.target, 'Unable to save configuration')
                        }
                    });
                }
            }
        });
    });