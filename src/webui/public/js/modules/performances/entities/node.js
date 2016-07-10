define(['application', 'backbone', 'lib/api', 'jquery'], function (App, Backbone, api, $) {
    return Backbone.Model.extend({
        config: {
            emotion: {
                label: 'Emotion',
                properties: ['duration', 'magnitude', 'emotion']
            },
            interaction: {
                label: 'Interaction',
                properties: ['duration', 'btree_mode', 'speech_event']
            },
            listening: {
                label: 'Listening',
                properties: ['duration']
            },
            expression: {
                label: 'Expression',
                properties: ['duration', 'expression', 'magnitude']
            },
            chat_pause: {
                label: 'Chat pause',
                properties: ['duration', 'message']
            },
            soma: {
                label: 'Soma State',
                properties: ['duration', 'soma']
            },
            gesture: {
                label: 'Animation',
                properties: ['speed', 'animation', 'magnitude']
            },
            head_rotation: {
                label: 'Head Tilt',
                properties: ['angle', 'speed']
            },
            kfanimation: {
                label: 'KF Animation',
                properties: ['kfanimation', 'fps', 'kfmode']
            },
            speech: {
                label: 'Speech',
                properties: ['language', 'speed', 'pitch', 'volume', 'text']
            },
            look_at: {
                label: 'Look at',
                properties: ['attention_region', 'crosshair', 'speed']
            },
            gaze_at: {
                label: 'Gaze at',
                properties: ['attention_region', 'crosshair', 'speed']
            },
            pause: {
                label: 'Pause',
                properties: ['topic', 'timeout']
            },
            default: {
                label: 'Node',
                properties: []
            }
        },
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
        getConfig: function () {
            var name = this.get('name');
            return this.config[name] || this.config['default'];
        },
        getLabel: function () {
            return this.getConfig().label;
        },
        getTitle: function () {
            var title = this.getLabel();

            if (this.get('text'))
                title = this.get('text');
            else if (this.get('emotion'))
                title = this.get('emotion');
            else if (this.get('gesture'))
                title = this.get('gesture');
            else if (this.get('expression'))
                title = this.get('expression');
            else if (this.get('animation') || this.get('kfanimation'))
                title = this.get('animation');

            return title
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
