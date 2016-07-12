define(['jquery', 'json_editor'], function ($, JSONEditor) {
    JSONEditor.defaults.editors.slider = JSONEditor.defaults.editors.number.extend({
        build: function () {
            JSONEditor.defaults.editors.number.prototype.build.call(this);
            var self = this,
                slider = $('<div>').slider({
                    range: "min",
                    min: this.options.schema.minimum || 0,
                    max: this.options.schema.maximum || 100,
                    step: this.options.schema.step || 0.1,
                    value: this.value,
                    slide: function (e, ui) {
                        $(self.input).val(ui.value).change();
                    }
                });

            $(this.input).after(slider).on('change', function () {
                slider.slider("value", $(self.input).val());
            });
        }
    });
});
