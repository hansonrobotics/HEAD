define(['application', 'backbone', 'lib/api', 'jquery'], function (App, Backbone, api, $) {
    return Backbone.Model.extend({
        initialize: function () {
            this.set('id', this.cid);
            this.on('change:el', this.updateEl);
        },
        call: function () {
            switch (this.get('name')) {
                case 'look_at':
                    api.setFaceTarget(this.get('x'), this.get('y'), this.get('z'));
                    break;
                case 'gaze_at':
                    api.setGazeTarget(this.get('x'), this.get('y'), this.get('z'));
                    break;
                case 'head_rotation':
                    api.set_head_rotation(this.get('angle'));
                    break;
            }
        },
        onDestroy: function () {
            this.removeEl();
        },
        updateEl: function () {
            if (this.previous('el')) $(this.previous('el')).remove();
        },
        removeEl: function () {
            if (this.get('el')) $(this.get('el')).remove();
        },
        toJSON: function () {
            var json = Backbone.Model.prototype.toJSON.call(this);
            if (this.get('el')) delete json['el'];

            return json;
        },
        destroy: function () {
            // remove an associated element
            if (this.collection) this.collection.remove(this);
            if (this.get('el')) $(this.get('el')).remove();
            this.unset('id');
            Backbone.Model.prototype.destroy.call(this);
        }
    });
});
