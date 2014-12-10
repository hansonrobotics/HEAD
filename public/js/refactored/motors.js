RosUI.motors = {
    init: function() {

    },

    load_page: function() {
        console.log('motors page load');
        var blenderMessage, blinkMessage, treeMessage;

        blenderMessage = new ROSLIB.Message({data: 'Dummy'});
        RosUI.ros.topics.cmdBlender.publish(blenderMessage);

        blinkMessage = new ROSLIB.Message({data: 'dmitry:stop'});
        RosUI.ros.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_off'});
        RosUI.ros.topics.cmdTree.publish(treeMessage);

        RoboInterface.set_expression("happy", 0);
        RoboInterface.pointHead({yaw: 0, pitch: 0, roll: 0});

    }
};
