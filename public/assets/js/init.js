require(["eventemitter"], function (EventEmitter2) {
    window.EventEmitter2 = EventEmitter2;
});

define(['jquery', 'lib/ros', 'lib/navigation', 'pages/status', 'pages/expressions',
        'pages/animations', 'pages/gestures', 'pages/interaction', 'pages/motors', 'jquery-ui', 'bootstrap'],
    function ($, ros, navigation, status, expressions, animations, gestures, interaction, motors) {
        ros.connect(function () {
            // init navigation
            navigation.init();

            // init pages
            status.init();
            expressions.init();
            motors.init();
            animations.init();
            gestures.init();
            interaction.init();
        });
    });
