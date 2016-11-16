var yaml = require('js-yaml'),
    mkdirp = require('mkdirp'),
    path = require('path'),
    fs = require('fs');

module.exports = {
    readFile: function (filename) {
        try {
            return yaml.load(fs.readFileSync(filename));
        } catch (err) {
            console.log('Error reading "' + filename + '":\n' + err)
            return null;
        }
    },
    writeFile: function (filename, data) {
        try {
            mkdirp.sync(path.dirname(filename));
            fs.writeFileSync(filename, yaml.dump(data));
            return true;
        } catch (err) {
            console.log('Error writing "' + filename + '":\n' + err)
            return false;
        }
    }
};
