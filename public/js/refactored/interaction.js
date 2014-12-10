RosUI.interaction = {
    init: function() {

    },

    load_page: function() {
        console.log('interactions page load');
        var blenderMessage, blinkMessage, treeMessage;

        blenderMessage = new ROSLIB.Message({data: 'TrackDev'});
        RosUI.ros.topics.cmdBlender.publish(blenderMessage);

        blinkMessage = new ROSLIB.Message({data: 'dmitry:start'});
        RosUI.ros.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_on'});
        RosUI.ros.topics.cmdTree.publish(treeMessage);

        RosUI.api.set_expression("happy", 0);
        RosUI.api.point_head({yaw: 0, pitch: 0, roll: 0});
    }
};
