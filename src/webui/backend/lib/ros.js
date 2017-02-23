var ROSLIB = require('roslib'),
    url = 'ws://localhost:9090',
    ros = new ROSLIB.Ros({
        url: url
    })

ros.on('connection', function() {
    console.log('Connected to websocket server.')
})

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: \n', error)
    console.log('Retrying in 2 seconds')
    setTimeout(function() {
        ros.connect(url)
    }, 2000)
})

ros.on('close', function() {
    console.log('Connection to websocket server closed.')
})

var self = module.exports = {
    instance: ros,
    services: {
        updateMotors: new ROSLIB.Service({
            ros: ros,
            name: '/webui/motors_controller/update_motors',
            serviceType: 'webui/UpdateMotors'
        }),
        performances: {
            reload_properties: new ROSLIB.Service({
                ros: ros,
                name: '/performances/reload_properties',
                serviceType: 'std_srvs/Trigger'
            }),
            set_properties: new ROSLIB.Service({
                ros: ros,
                name: '/performances/set_properties',
                serviceType: 'performances/SetProperties'
            }),
            load: new ROSLIB.Service({
                ros: ros,
                name: '/performances/load',
                messageType: 'performances/Load'
            }),
            load_sequence: new ROSLIB.Service({
                ros: ros,
                name: '/performances/load_sequence',
                messageType: 'performances/LoadSequence'
            }),
            load_performance: new ROSLIB.Service({
                ros: ros,
                name: '/performances/load_performance',
                messageType: 'performances/LoadPerformance'
            }),
            current: new ROSLIB.Service({
                ros: ros,
                name: '/performances/current',
                messageType: 'performances/Current'
            }),
            run: new ROSLIB.Service({
                ros: ros,
                name: '/performances/run',
                messageType: 'performances/Run'
            }),
            run_by_name: new ROSLIB.Service({
                ros: ros,
                name: '/performances/run_by_name',
                messageType: 'performances/RunByName'
            }),
            run_full_performance: new ROSLIB.Service({
                ros: ros,
                name: '/performances/run_full_performance',
                messageType: 'performances/RunByName'
            }),
            stop: new ROSLIB.Service({
                ros: ros,
                name: '/performances/stop',
                messageType: 'performances/Stop'
            }),
            pause: new ROSLIB.Service({
                ros: ros,
                name: '/performances/pause',
                messageType: 'performances/Pause'
            }),
            resume: new ROSLIB.Service({
                ros: ros,
                name: '/performances/resume',
                messageType: 'performances/Resume'
            })
        },
        updateExpressions: new ROSLIB.Service({
            ros: ros,
            name: '/webui/motors_controller/update_expressions',
            serviceType: 'webui/UpdateExpressions'
        })
    },
    topics: {
        lookAt: new ROSLIB.Topic({
            ros: ros,
            name: '/camera/face_locations',
            messageType: 'pi_face_tracker/Faces'
        })
    },
    updateMotors: function(robot_name, motors) {
        this.services.updateMotors.callService({
            robot_name: robot_name,
            motors: JSON.stringify(motors)
        }, function(res) {
            console.log(res)
        })
    },
    lookAt: function(point) {
        this.topics.lookAt.publish(new ROSLIB.Message({
            faces: [{
                id: 1,
                point: point,
                attention: 0.99
            }]
        }))
    },
    performances: {
        reloadProperties: function() {
            self.services.performances.reload_properties.callService()
        },
        setProperties: function(id, properties) {
            self.services.performances.set_properties.callService({
                id: id,
                properties: properties
            }, function(response) {
                console.log(response)

            })
        }
    },
    updateExpressions: function(robot_name) {
        this.services.updateExpressions.callService({
            robot_name: robot_name,
        }, function(res) {
            console.log(res)
        })
    },
}
