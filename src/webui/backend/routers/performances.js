let express = require('express'),
    router = express.Router(),
    ros = require('../lib/ros')

router.post('/reload_properties', function(req, res) {
    ros.performances.reloadProperties({
        success: function() {
            res.json({success: true})
        },
        error: function() {
            res.json({success: false})
        }
    })
})

router.post('/set_properties', function(req, res) {
    ros.performances.setProperties(req.body['id'], req.body['properties'], {
        success: function(response) {
            res.json({success: response.success})
        },
        error: function() {
            res.json({success: false})
        }
    })
})

router.post('/load', function(req, res) {
    ros.services.performances.load.callService({
        id: req.body['id']
    }, function(response) {
        let success = response.success
        res.json({success: success, performance: success ? JSON.parse(response.performance) : null})
    }, function() {
        res.json({success: false, performance: null})
    })
})

router.post('/load_performance', function(req, res) {
    ros.services.performances.load_performance.callService({
        performance: JSON.stringify(req.body['performance'])
    }, function(response) {
        res.json({success: response.success})
    }, function() {
        res.json({success: false})
    })
})

router.post('/unload', function(req, res) {
    ros.services.performances.unload.callService({},
        function() {
            res.json({success: true})
        }, function() {
            res.json({success: false})
        })
})

router.post('/run_by_name', function(req, res) {
    ros.services.performances.run_by_name.callService({
        id: req.body['id']
    }, function(response) {
        res.json({success: response.success})
    }, function() {
        res.json({success: false})
    })
})

router.post('/run_full_performance', function(req, res) {
    ros.services.performances.run_full_performance.callService({
        id: req.body['id']
    }, function(response) {
        res.json({success: response.success})
    }, function() {
        res.json({success: false})
    })
})

router.get('/current', function(req, res) {
    ros.services.performances.current.callService({}, function(response) {
        res.json({
            running: response.running,
            current_time: response.current_time,
            performance: JSON.parse(response.performance)
        })
    }, function() {
        res.json({
            running: false,
            current_time: 0,
            performances: []
        })
    })
})

router.post('/run', function(req, res) {
    ros.services.performances.run.callService({
        startTime: parseFloat(req.body['start_time']) || 0
    }, function(response) {
        res.json({success: response.success})
    }, function() {
        res.json({success: false})
    })
})

router.post('/resume', function(req, res) {
    ros.services.performances.resume.callService({}, function(response) {
        res.json({success: response.success, time: response.time})
    }, function() {
        res.json({success: false, time: null})
    })
})

router.post('/pause', function(req, res) {
    ros.services.performances.pause.callService({}, function(response) {
        res.json({success: response.success, time: response.time})
    }, function() {
        res.json({success: false, time: null})
    })
})


router.post('/stop', function(req, res) {
    ros.services.performances.stop.callService({}, function(response) {
        res.json({success: response.success, time: response.time})
    }, function() {
        res.json({success: false, time: null})
    })
})

module.exports = router
