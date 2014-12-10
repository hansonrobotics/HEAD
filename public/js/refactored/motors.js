RosUI.motors = {
    init: function() {

    },

    loadPage: function() {
        console.log('motors page load');
        var blenderMessage, blinkMessage, treeMessage;

        blenderMessage = new ROSLIB.Message({data: 'Dummy'});
        RosUI.api.topics.cmdBlender.publish(blenderMessage);

        blinkMessage = new ROSLIB.Message({data: 'dmitry:stop'});
        RosUI.api.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_off'});
        RosUI.api.topics.cmdTree.publish(treeMessage);

        RoboInterface.set_expression("happy", 0);
        RoboInterface.pointHead({yaw: 0, pitch: 0, roll: 0});

    }
};
