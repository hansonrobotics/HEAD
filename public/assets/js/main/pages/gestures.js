define(['jquery', './../lib/api'], function ($, api) {
    var gestures = {
        config: {},
        init: function () {
            gestures.createGestureButtons();
            gestures.createEmotionSliders();

            gestures.demo.init();
        },
        loadPage: function () {
            api.blenderMode.enable();
        },
        createGestureButtons: function () {
            api.getAvailableGestures(function (gestures) {
                $.each(gestures, function () {
                    var gesture = this;

                    var button = $('<button type="button" class="btn btn-default app-gesture-button" data-gesture="' +
                    gesture + '">' + gesture + '</button>');

                    $(button).click(function () {
                        $('#app-gesture-buttons button').removeClass('active');
                        $(this).addClass('active');

                        api.setGesture(gesture);
                    });

                    $('#app-gesture-buttons')
                        .append(button);
                });
            });
        },
        createEmotionSliders: function () {
            api.getAvailableEmotionStates(function (emotions) {
                $.each(emotions, function () {
                    gestures.addEmotionSlider(this);
                });
            });
        },
        addEmotionSlider: function (name, value) {
            if (typeof value == "undefined")
                value = 0;

            var sliderBlock = $("#app-emotion-slider-template").clone().attr('data-emotion', name);
            sliderBlock.removeAttr("id"); //Removing app-slider-template id

            sliderBlock.find(".app-slider-label-left").text(name);
            $(".app-slider-value-container", sliderBlock).html("(" + value + "%)");

            //Create slider
            var slider = sliderBlock.find(".app-slider");
            slider.slider({
                range: "min",
                value: 0,
                min: 0,
                max: 100,
                change: (function () {
                    return function (e, ui) {
                        $('.app-slider-value-container', sliderBlock).html("(" + ui.value + "%)");

                        api.setEmotion(name, ui.value / 100);
                    }
                })()
            });

            sliderBlock.removeClass("hidden");
            sliderBlock.appendTo("#app-emotion-sliders");
        },
        updateSlider: function (name, value) {
            var container = $('.app-emotion-slider-container[data-emotion="' + name + '"]');

            $(".app-slider", container).slider("value", value).trigger('slide');
            $(".app-slider-value-container", container).text('(' + value + '%)');
        },
        demo: {
            init: function () {
                $('#app-gesture-demo-start').click(function () {
                    $(this).addClass('active');
                    $('#app-gesture-demo-stop').removeClass('active');

                    gestures.demo.enabled = true;
                });

                $('#app-gesture-demo-stop').click(function () {
                    $(this).addClass('active');
                    $('#app-gesture-demo-start').removeClass('active');

                    gestures.demo.enabled = false;
                }).click();

                setInterval(function () {
                    if (gestures.demo.enabled)
                        gestures.demo.setRandomGesture();
                }, 2000);
            },
            setRandomGesture: function () {
                var emotionSliders = $('.app-emotion-slider-container[data-emotion]'),
                    randomSlider = $(emotionSliders).get(Math.floor(Math.random() * $(emotionSliders).length));

                $.each(emotionSliders, function () {
                    if (this != randomSlider)
                        gestures.updateSlider($(this).data('emotion'), 0)
                });

                gestures.updateSlider($(randomSlider).data('emotion'), Math.floor(Math.random() * 30) + 40)

                var gestureButtons = $('#app-gesture-buttons button'),
                    randomButton = $(gestureButtons).get(Math.floor(Math.random() * $(gestureButtons).length));

                $(randomButton).click();
            },
            enable: function () {
                $('#app-gesture-demo-start').click();
            },
            disable: function () {
                $('#app-gesture-demo-stop').click();
            }
        }
    };

    return gestures;
});
