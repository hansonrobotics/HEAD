define(['application', 'backbone', 'lib/api'], function (App, Backbone, api) {
    return Backbone.Model.extend({
        call: function () {
            var self = this;
            switch (this.get('name')) {
                case 'look_at':
                    api.setFaceTarget(this.get('x'), this.get('y'), this.get('z'));
                    break;
                case 'gaze_at':
                    api.setGazeTarget(this.get('x'), this.get('y'), this.get('z'));
                    break;
                case 'head_rotation':
                    api.set_head_rotation(this.get('angle'))
                    break;
            }
        },
        toJSON: function () {
            var json = Backbone.Model.prototype.toJSON.call(this);
            if (this.get('el')) delete json['el'];

            return json;
        },
        destroy: function () {
            // remove an associated element
            if (this.get('el'))
                $(this.get('el')).remove();

            Backbone.Model.prototype.destroy.call(this);
        }
    });
});
