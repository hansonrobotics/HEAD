RosUI = {
    topics: {},
    initPage: function () {
        // send appropriate messages
        var blenderMessage, blinkMessage, treeMessage;
        switch ($('.app-change-page.active').attr('id')) {
            case 'app-expressions-link':
                console.log('expressions page load');

                // send messages
                blenderMessage = new ROSLIB.Message({data: 'Dummy'});
                RosUI.topics.cmdBlender.publish(blenderMessage);

                blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
                RosUI.topics.cmdBllink.publish(blenderMessage);

                treeMessage = new ROSLIB.Message({data: 'btree_off'});
                RosUI.topics.cmdTree.publish(treeMessage);

                $('.expression-button.active').removeClass('active');
                RoboInterface.makeFaceExpr("happy", 0);
                RoboInterface.pointHead({yaw: 0, pitch: 0, roll: 0});

                break;
            case 'app-motors-link':
                console.log('motors page load');

                blenderMessage = new ROSLIB.Message({data: 'Dummy'});
                RosUI.topics.cmdBlender.publish(blenderMessage);

                blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
                RosUI.topics.cmdBllink.publish(blinkMessage);

                treeMessage = new ROSLIB.Message({data: 'btree_off'});
                RosUI.topics.cmdTree.publish(treeMessage);

                RoboInterface.makeFaceExpr("happy", 0);
                RoboInterface.pointHead({yaw: 0, pitch: 0, roll: 0});

                break;
            case 'app-animations-link':
                console.log('animations page load');

                blenderMessage = new ROSLIB.Message({data: 'Animations'});
                RosUI.topics.cmdBlender.publish(blenderMessage);

                blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
                RosUI.topics.cmdBllink.publish(blinkMessage);

                treeMessage = new ROSLIB.Message({data: 'btree_off'});
                RosUI.topics.cmdTree.publish(treeMessage);

                RoboInterface.makeFaceExpr("happy", 0);
                RoboInterface.pointHead({yaw: 0, pitch: 0, roll: 0});

                $('[data-cmd="stop"]').click();

                break;
            case 'app-interactions-link':
                console.log('interactions page load');

                blenderMessage = new ROSLIB.Message({data: 'TrackDev'});
                RosUI.topics.cmdBlender.publish(blenderMessage);

                blinkMessage = new ROSLIB.Message({data: 'arthur:start'});
                RosUI.topics.cmdBllink.publish(blinkMessage);

                treeMessage = new ROSLIB.Message({data: 'btree_on'});
                RosUI.topics.cmdTree.publish(treeMessage);

                RoboInterface.makeFaceExpr("happy", 0);
                RoboInterface.pointHead({yaw: 0, pitch: 0, roll: 0});

                break;
        }
    }
};

$(document).ready(function () {
    RosUI.ros = RoboInterface.connect(websocketAddress());
    RosUI.ros.$.on('connection', function () {
        RosUI.topics = {
            cmdBlender: new ROSLIB.Topic({
                ros: ros,
                name: '/cmd_blendermode',
                messageType: 'std_msgs/String'
            }),
            cmdBllink: new ROSLIB.Topic({
                ros: ros,
                name: '/arthur/cmd_blink',
                messageType: 'std_msgs/String'
            }),
            cmdTree: new ROSLIB.Topic({
                ros: ros,
                name: '/arthur/behavior_switch',
                messageType: 'std_msgs/String'
            }),
            speech_topic: new ROSLIB.Topic({
                ros: ros,
                name: '/arthur/chatbot_speech',
                messageType: 'chatbot/ChatMessage'
            })
        };

        var anchor = window.location.hash,
            pageLink = $('.app-change-page[href="' + anchor + '"]');

        if ($(pageLink).length)
            $(pageLink).click();
        else
            $('.app-change-page.active').click();
    });

    $('.app-change-page').click(function () {
        var pageEl = $(this).data('page');
        $('.app-change-page').removeClass('active');
        $(this).addClass('active');

        if ($('.app-page:visible').length == 0) {
            $(pageEl).fadeIn();
        } else {
            $('.app-page:visible').fadeOut(400, function () {
                $(pageEl).fadeIn();
            });
        }
        $('#app-title').html($(this).html());
        $('.navbar-toggle:visible:not(.collapsed)').click();

        if (typeof ros != 'undefined') {
            RosUI.initPage();
        }
    });

    $('#notifications .label').click(function () {
        location.reload();
    });
});
