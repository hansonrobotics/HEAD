define(['bootbox', 'icheck', 'icheck/skins/polaris/polaris.css'], function(bootbox, icheck) {
    return Marionette.View.extend({
        template: require('./templates/item.tpl'),
        tagName: 'li',
        className: 'app-performance list-group-item',
        ui: {
            playButton: '.app-play',
            description: '.app-desc',
            duration: '.app-duration',
            removeButton: '.app-remove-button',
            dragHandle: '.app-drag-handle',
            enableCheckbox: '.app-enable-checkbox'
        },
        initialize: function(options) {
            this.mergeOptions(options, ['layoutView', 'readonly', 'queueView'])
        },
        events: {
            'click @ui.playButton': 'play',
            'click @ui.removeButton': 'remove',
            'click': 'setItemTime'
        },
        modelEvents: {
            'change': 'render',
            'change:duration': 'render'
        },
        onRender: function() {
            if (this.readonly)
                this.ui.dragHandle.hide()

            let self = this,
                performance = this.model.get('performance')

            this.ui.description.html(performance.getDescription())
            this.ui.duration.html(performance.getDuration().toFixed(2))
            if (!this.queueView.playEnabled)
                this.ui.playButton.hide()

            if (this.readonly) this.ui.removeButton.hide()

            let enabled = performance.get('enabled')
            enabled = typeof enabled === 'undefined' || enabled

            this.ui.enableCheckbox.iCheck({
                checkboxClass: 'icheckbox_polaris',
                labelHover: true
            }).iCheck(enabled ? 'check' : 'uncheck').on('ifToggled', function() {
                performance.save({'enabled': self.ui.enableCheckbox.is(':checked')}, {
                    success: function() {
                        self.layoutView.updateTimeline()
                    }
                })
            })
        },
        play: function(e) {
            e.stopPropagation()
            this.layoutView.play(this.model)
        },
        remove: function(e) {
            this.layoutView.remove(this.model)
            return false
        },
        setItemTime: function() {
            this.layoutView.setItemTime(this.model)
        },
        enablePlay: function() {
            this.ui.playButton.fadeIn()
        },
        disablePlay: function() {
            this.ui.playButton.fadeOut()
        }
    })
})
