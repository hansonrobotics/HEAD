var ROSLIB = require('roslib'),
    url = 'ws://localhost:9090',
    ros = new ROSLIB.Ros({
        url: url
    });

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: \n', error);
    console.log('Retrying in 2 seconds')
    setTimeout(function () {
        ros.connect(url);
    }, 2000);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

module.exports = {
    instance: ros,
    services: {
        updateMotors: new ROSLIB.Service({
            ros : ros,
            name : '/webui/motors_controller/update_motors',
            serviceType : 'webui/UpdateMotors'
        })
    },
    topics: {
        lookAt: new ROSLIB.Topic({
            ros : ros,
            name : '/camera/face_locations',
            messageType : 'pi_face_tracker/Faces'
        })
    },
    updateMotors: function (robot_name, motors) {
        this.services.updateMotors.callService({
            robot_name: robot_name,
            motors: JSON.stringify(motors)
        }, function (res) {
            console.log(res);
        });
    },
    lookAt: function (point) {
        this.topics.lookAt.publish(new ROSLIB.Message({faces: [{
            id: 1,
            point: point,
            attention: 0.99
        }]}));
    }
};
