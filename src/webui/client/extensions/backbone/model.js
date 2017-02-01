Backbone.Model.prototype.toJSON = function() {
    let json = _.clone(this.attributes)
    for (let attr in json) {
        if ((json[attr] instanceof Backbone.Model) || (json[attr] instanceof Backbone.Collection)) {
            json[attr] = json[attr].toJSON()
        }
    }
    return json
}