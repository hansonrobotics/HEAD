define(['application', 'marionette', 'tpl!./templates/settings.tpl', 'json_editor'],
    function (App, Marionette, template, JSONEditor) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                settings: '.app-settings',
                save: '.app-save'
            },
            events: {
                'click @ui.save': 'save'
            },
            modelEvents: {
                change: 'setConfig'
            },
            onRender: function () {
                this.editor = new JSONEditor(this.ui.settings.get(0), {
                    form_name_root: 'config',
                    theme: 'bootstrap3',
                    schema: this.model.toJSON(),
                    disable_collapse: true,
                    disable_edit_json: true,
                    disable_properties: true,
                    iconlib: "bootstrap3"
                });
            },
            setConfig: function () {
                if (this.editor)
                    this.editor.setValue(this.model.toJSON());
            },
            save: function (e) {
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
        });
    });