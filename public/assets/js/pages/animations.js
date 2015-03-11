// Deprecated
define(['jquery', 'lib/api'], function ($, api) {
    var animations = {
        init: function () {
            //require(['ros_ui', 'apps/motor/motor_app'], function (RosUi) {
            //    RosUi.start();
            //});
        },
        loadPage: function () {
            api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
            api.pointHead();
        }
    };

    return animations;
});
