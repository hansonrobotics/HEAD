define(['jquery', 'lib/api', 'lib/ros', 'lib/utilities'], function ($, api, ros, utilities, motorsLayout) {
    var motors = {
        init: function () {
            require(['application', 'modules/motor/motor_app']);
        },
        loadPage: function () {
            api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
            api.pointHead();
        }
    };

    return motors;
});
