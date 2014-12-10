RosUI.interaction = {
    init: function() {

    },

    loadPage: function() {
        console.log('interactions page load');
        var blenderMessage, blinkMessage, treeMessage;

        blenderMessage = new ROSLIB.Message({data: 'TrackDev'});
        RosUI.api.topics.cmdBlender.publish(blenderMessage);

        blinkMessage = new ROSLIB.Message({data: 'dmitry:start'});
        RosUI.api.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_on'});
        RosUI.api.topics.cmdTree.publish(treeMessage);

        RoboInterface.set_expression("happy", 0);
        RoboInterface.pointHead({yaw: 0, pitch: 0, roll: 0});
    }
};
