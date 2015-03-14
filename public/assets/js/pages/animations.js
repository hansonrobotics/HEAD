// Deprecated
define(['jquery', 'lib/api'], function ($, api) {
    var animations = {
        init: function () {
            require(['application', 'modules/animations/animations_app']);
        },
        loadPage: function () {
            api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
            api.pointHead();
        }
    };

    return animations;
});
