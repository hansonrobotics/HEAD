let bootbox = require('bootbox')

module.exports = Marionette.View.extend({
    template: require('./templates/item.tpl'),
    tagName: 'li',
    className: 'app-performance list-group-item',
    ui: {
        playButton: '.app-play',
        description: '.app-desc',
        duration: '.app-duration',
        removeButton: '.app-remove-button'
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
        let performance = this.model.get('performance')
        this.ui.description.html(performance.getDescription())
        this.ui.duration.html(performance.getDuration().toFixed(2))
        if (!this.queueView.playEnabled)
            this.ui.playButton.hide()

        if (this.readonly) this.ui.removeButton.hide()
    },
    play: function(e) {
        e.stopPropagation()
        this.layoutView.play(this.model)
    },
    remove: function() {
        this.layoutView.remove(this.model)
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
