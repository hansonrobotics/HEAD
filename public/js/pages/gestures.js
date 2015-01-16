RosUI.gestures = {
    config: {},
    init: function () {
        RosUI.gestures.createGestureButtons();
        RosUI.gestures.createEmotionSliders();

        RosUI.gestures.demo.init();
    },
    loadPage: function () {
        RosUI.api.blenderMode.enable();
    },
    createGestureButtons: function () {
        RosUI.api.getAvailableGestures(function(gestures) {
            $.each(gestures, function () {
                var gesture = this;

                var button = $('<button type="button" class="btn btn-default app-gesture-button" data-gesture="' +
                gesture + '">' + gesture + '</button>');

                $(button).click(function () {
                    $('#app-gesture-buttons button').removeClass('active');
                    $(this).addClass('active');

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
            change: (function () {
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

        $(".slider", container).slider("value", value).trigger('slide');
        $(".app-value", container).text('(' + value + '%)');
    },
    demo: {
        init: function() {
            $('#app-gesture-demo-start').click(function() {
                $(this).addClass('active');
                $('#app-gesture-demo-stop').removeClass('active');

                RosUI.gestures.demo.enabled = true;
            });

            $('#app-gesture-demo-stop').click(function() {
                $(this).addClass('active');
                $('#app-gesture-demo-start').removeClass('active');

                RosUI.gestures.demo.enabled = false;
            }).click();

            setInterval(function() {
                if (RosUI.gestures.demo.enabled)
                    RosUI.gestures.demo.setRandomGesture();
            }, 2000);
        },
        setRandomGesture: function() {
            var emotionSliders = $('.uiBlock[data-emotion]'),
                randomSlider = $(emotionSliders).get(Math.floor(Math.random() * $(emotionSliders).length));

            $.each(emotionSliders, function() {
                if (this != randomSlider)
                    RosUI.gestures.updateSlider($(this).data('emotion'), 0)
            });

            RosUI.gestures.updateSlider($(randomSlider).data('emotion'), Math.floor(Math.random() * 30) + 40)

            var gestureButtons = $('#app-gesture-buttons button'),
                randomButton   = $(gestureButtons).get(Math.floor(Math.random() * $(gestureButtons).length));

            $(randomButton).click();
        },
        enable: function() {
            $('#app-gesture-demo-start').click();
        },
        disable: function() {
            $('#app-gesture-demo-stop').click();
        }
    }
};
