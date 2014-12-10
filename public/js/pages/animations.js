init_animations = function () {
    // ROS object to connect to ROS

    var animations = [];
    var cmdAnimations;
    var lastAnim = 'happy-1'
    // functions
    // Starts Animations mode in blender

    // Get animations
    function getAnimations() {
        console.log('anims');
        var anims = new ROSLIB.Topic({
            ros: RosUI.api.ros,
            name: '/animations_list',
            messageType: 'robo_blender/animations_list'
        });
        anims.subscribe(function (msg) {
            animations = msg.actions;
            anims.unsubscribe();
        });
    }

    function addButtons() {
        cmdAnimations = new ROSLIB.Topic({
            ros: RosUI.api.ros,
            name: '/cmd_animations',
            messageType: 'std_msgs/String'
        });

        $('.app-animation-button').click(function () {
            var data = $(this).data('animation');
            console.log(data);
            var msg = new ROSLIB.Message({
                data: 'play:' + data
            });

            lastAnim = data;
            cmdAnimations.publish(msg);

            $('[data-cmd="play"]').click();
        });
        $('.command').click(function () {
            var data = $(this).data('cmd');

            if (data == 'play') {
                data = data + ':' + lastAnim
            }
            var msg = new ROSLIB.Message({
                data: data
            });
            cmdAnimations.publish(msg);
        });
        $('#cmd_stop').click();

    }

    // Find out exactly when we made a connection.
    RosUI.api.ros.on('connection', function () {
        console.log('Connection made!');
        setTimeout(function () {
            addButtons()
        }, 1000);

    });

    var loopOn = 0;

    $('#demo-loop').find('.btn').click(function () {
        $('#demo-loop').find('.btn').removeClass('active');
        $(this).addClass('active');

        var elements = $(".app-optional");
        switch ($(this).attr('id')) {
            case 'loop-on':
                loopOn = 1;
                elements.hide();
                break;
            case 'loop-off':
                loopOn = 0;
                elements.show();
                $('.command.btn.active').click();
                break;
        }
    });

    //Demo loop
    jQuery.fn.random = function () {
        var randomIndex = Math.floor(Math.random() * this.length);
        return jQuery(this[randomIndex]);
    };
    var demo = setInterval(function () {
        if (loopOn > 0) {
            $('.app-animation-button').random().click();
        }
    }, 1000);
};
