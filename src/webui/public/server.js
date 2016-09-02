var express = require('express');
var app = express();

express.static();

// respond with "hello world" when a GET request is made to the homepage
app.get('/', function(req, res) {
    res.send('hello world');
});

app.listen(8003);
