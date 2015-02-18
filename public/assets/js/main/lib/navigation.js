define(['jquery', './api', 'main/pages/status', './../pages/expressions',
    './../pages/animations', './../pages/gestures', './../pages/interaction', './../pages/motors'],
    function ($, api, status, expressions, animations, gestures, interaction, motors) {
    var navigation = {
        init: function() {
            // bind page change to navigation links
            $('.app-change-page').click(function () {
                var page_selector = $(this).data('page');

                // change active page link
                $('.app-change-page').removeClass('active');
                $(this).addClass('active');

                if ($('.app-page:visible').length == 0) {
                    // show the page if no pages are showed yet
                    $(page_selector).fadeIn();
                } else {
                    // hide previous page and show the current
                    $('.app-page:visible').fadeOut(400, function () {
                        $(page_selector).fadeIn();
                    });
                }

                // change title
                $('#app-title').html($(this).html());

                // load page
                if (typeof api.ros != 'undefined')
                    navigation.reload();
            });

            // reload when clicking on error message
            $('#notifications .label').click(function () {
                location.reload();
            });

            // pick up current page from the hash
            var anchor = window.location.hash,
                pageLink = $('.app-change-page[href="' + anchor + '"]');

            if ($(pageLink).length) {
                // load page if found
                $(pageLink).click();
            } else {
                // load current active page by default
                $('.app-change-page.active').click();
            }
        },
        reload: function () {
            gestures.demo.disable();

            switch ($('.app-change-page.active').attr('id')) {
                case 'app-status-link':
                    status.loadPage();
                    break;
                case 'app-expressions-link':
                    expressions.loadPage();
                    break;
                case 'app-motors-link':
                    motors.loadPage();
                    break;
                case 'app-animations-link':
                    animations.loadPage();
                    break;
                case 'app-interactions-link':
                    interaction.loadPage();
                    break;
                case 'app-gestures-link':
                    gestures.loadPage();
                    break;
            }
        }
    };

    return navigation;
});
