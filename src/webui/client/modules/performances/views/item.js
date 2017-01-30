module.exports = Marionette.View.extend({
	template: require('./templates/item.tpl'),
	tagName: 'li',
	className: 'app-performance list-group-item',
	ui: {
		playButton: '.app-play',
		name: '.app-name',
		description: '.app-desc',
		duration: '.app-duration',
		editButton: '.app-edit-button',
		removeButton: '.app-remove-button'
	},
	initialize: function(options) {
		this.mergeOptions(options, ['layoutView', 'readonly']);
	},
	events: {
		'click @ui.playButton': 'play',
		'click @ui.editButton': 'edit',
		'click @ui.removeButton': 'remove',
		'click': 'setTime'
	},
	modelEvents: {
		'change': 'render',
		'change:duration': 'render'
	},
	onRender: function() {
		let performance = this.model.get('performance');
		this.ui.name.html(performance.get('name'));
		this.ui.description.html(performance.getDescription());
	    this.ui.duration.html(performance.getDuration().toFixed(2));
	},
	play: function(e) {
        e.stopPropagation();
        this.layoutView.play(this.model.get('performance'));
	},
	edit: function() {
        this.layoutView.editPerformance(this.model.get('performance'));
	},
	remove: function() {
        this.layoutView.queueCollection.remove(this.model);
	},
	setTime: function() {
        this.layoutView.setTime(this.model.get('performance'));
	}
});
