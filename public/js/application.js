RosUI = {
    init: function() {
        // init rosbridge
        RosUI.api.init(function() {
            // init navigartion
            RosUI.navigation.init();

            RosUI.expressions.init();
            RosUI.motors.init();
            RosUI.animations.init();
            RosUI.interaction.init();

            init_interaction();
            init_motors();
        });
    },
    topics: {}
};

$(document).ready(function () {
    RosUI.init();
});
