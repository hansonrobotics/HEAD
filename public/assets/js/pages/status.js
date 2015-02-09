RosUI.status = {
    config: {
    },
    init: function () {
    },
    loadPage: function () {
        var blinkMessage, treeMessage;

        RosUI.api.blenderMode.disable();

        blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
        RosUI.ros.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_off'});
        RosUI.ros.topics.cmdTree.publish(treeMessage);

        RosUI.api.setExpression("happy", 0);
        RosUI.api.pointHead();
    }
};
