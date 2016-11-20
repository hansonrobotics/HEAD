define(["application", "./templates/frame.tpl"], function (App, template) {
    return Marionette.View.extend({
        template: template,
        tagName: 'li',
        ui: {
            name: '.app-name',
            frames: '.app-frames-input',
            speed: '.app-speed',
            acceleration: '.app-acceleration',
            selectButton: '.app-select-frame',
            deleteButton: '.app-delete-frame',
            copyButton: '.app-copy-frame'
        },
        events: {
            'click @ui.selectButton': 'selectFrame',
            'click @ui.deleteButton': 'deleteFrame',
            'click @ui.copyButton': 'copyFrame',
            'change @ui.name': 'updateFrame',
            'change @ui.frames': 'updateFrame',
            'change @ui.speed': 'updateFrame',
            'change @ui.acceleration': 'updateFrame'
        },
        modelEvents: {
            change: 'modelChanged'
        },
        onRender: function () {
            this.modelChanged();
        },
        modelChanged: function () {
            this.ui.name.val(this.model.get('name'));
            this.ui.frames.val(this.model.get('frames'));
            this.ui.speed.val(this.model.get('speed'));
            this.ui.acceleration.val(this.model.get('acceleration'));
        },
        selectFrame: function () {
            $('.app-select-frame, .app-frames li').removeClass('active');
            this.ui.selectButton.addClass('active');
            this.$el.addClass('active');
            View.trigger('frame_selected', this.model);
        },
        updateFrame: function () {
            this.model.set('name', this.ui.name.val());
            this.model.set('frames', parseInt(this.ui.frames.val()));
            this.model.set('speed', parseFloat(this.ui.speed.val()));
            this.model.set('acceleration', parseFloat(this.ui.acceleration.val()));
        },
        deleteFrame: function () {
            if (confirm('Are you sure you want to delete this frame?'))
                this.model.destroy();
        },
        copyFrame: function () {
            View.trigger('copy_frame', this.model);
        }
    });

});
