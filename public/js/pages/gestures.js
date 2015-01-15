RosUI.gestures = {
    config: {},
    init: function () {
        RosUI.gestures.createGestureButtons();
        RosUI.gestures.createEmotionSliders();
    },
    loadPage: function () {

    },
    createGestureButtons: function () {
        RosUI.api.getAvailableGestures(function(gestures) {
            $.each(gestures, function () {
                var gesture = this;

                var button = $('<button type="button" class="btn btn-default app-gesture-button" data-gesture="' +
                gesture + '">' + gesture + '</button>');

                $(button).click(function () {
                    RosUI.api.setGesture(gesture);
                });

                $('#app-gesture-buttons')
                    .append(button);
            });
        });
    },
    createEmotionSliders: function () {
        RosUI.api.getAvailableEmotionStates(function(emotions) {
            $.each(emotions, function () {
                RosUI.gestures.addEmotionSlider(this);
            });
        });
    },
    addEmotionSlider: function (name, value) {
        if (typeof value == "undefined")
            value = 0;

        var sliderBlock = $("#sliderTemplate").clone().attr('data-emotion', name);
        sliderBlock.removeAttr("id"); //Removing sliderTemplate id

        sliderBlock.find(".slNameL").text(name);
        $(".app-value", sliderBlock).html("(" + value + "%)");

        //Create slider
        var slider = sliderBlock.find(".slider");
        slider.slider({
            range: "min",
            value: 0,
            min: 0,
            max: 100,
            slide: (function () {
                return function (e, ui) {
                    $('.app-value', sliderBlock).html("(" + ui.value + "%)");

                    RosUI.api.setEmotion(name, ui.value / 100);
                }
            })()
        });

        sliderBlock.removeClass("hidden");
        sliderBlock.appendTo("#app-emotion-sliders");
    },
    updateSlider: function (name, value) {
        var container = $('.uiBlock[data-emotion="' + name + '"]');

        $(".slider", container).slider({
            value: value,
            animate: "normal"
        });

        $(".app-value", container).text('(' + value + '%)');
    }
};
