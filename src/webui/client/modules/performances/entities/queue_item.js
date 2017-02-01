let Performance = require('./performance')

module.exports = Backbone.Model.extend({
    initialize: function() {
        this.on('change:performance', this.updatePerformance)
        this.updatePerformance()
    },
    updatePerformance: function() {
        let self = this,
            performance = this.get('performance'),
            previous = this.previous('performance')

        if (performance instanceof Performance)
            this.listenTo(performance, 'change', function() {
                self.trigger('change')
            })

        if (previous instanceof Performance)
            this.stopListening(previous)
    },
    getStartTime: function() {
        let self = this,
            startTime = 0

        if (this.collection) {
            this.collection.find(function(i) {
                let found = i === self
                if (!found) startTime += i.get('performance').getDuration()
                return found
            })
        }

        return startTime
    }
})
