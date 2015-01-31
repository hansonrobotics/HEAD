RosUI.animations = {
    init: function () {
        var last_animation = 'happy-1';

        $('.app-animation-button').click(function () {
            last_animation = $(this).data('animation');

            // this will call RosUI.api.playAnimation
            $('[data-cmd="play"]').click();
        });

        $('.command').click(function () {
            var data = $(this).data('cmd');

            if (data == 'play')
                RosUI.api.playAnimation(last_animation);
        });

        $('#cmd_stop').click();

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
            return $(this[randomIndex]);
        };

        var demo = setInterval(function () {
            if (loopOn > 0) {
                $('.app-animation-button').random().click();
            }
        }, 1000);
    },
    loadPage: function () {
        var blenderMessage, blinkMessage, treeMessage;

        //blenderMessage = new ROSLIB.Message({data: 'Animations'});
        //RosUI.ros.topics.cmdBlender.publish(blenderMessage);

        RosUI.api.blenderMode.disable();

        blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
        RosUI.ros.topics.cmdBllink.publish(blinkMessage);

        treeMessage = new ROSLIB.Message({data: 'btree_off'});
        RosUI.ros.topics.cmdTree.publish(treeMessage);

        RosUI.api.setExpression("happy", 0);
        RosUI.api.pointHead({yaw: 0, pitch: 0, roll: 0});

        $('[data-cmd="stop"]').click();
    }
};