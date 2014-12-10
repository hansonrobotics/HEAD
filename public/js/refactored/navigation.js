RosUI.navigation = {
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
            if (typeof ros != 'undefined')
                RosUI.navigation.reload();
        });

        // reload when clicking on error message
        $('#notifications .label').click(function () {
            location.reload();
        });

        // pick up current page from the hash
        var anchor = window.location.hash,
            pageLink = $('.app-change-page[href="' + anchor + '"]');

        if ($(pageLink).length) {
            // load page if has is valid
            $(pageLink).click();
        } else {
            // load current active page by default
            $('.app-change-page.active').click();
        }
    },
    reload: function () {
        switch ($('.app-change-page.active').attr('id')) {
            case 'app-expressions-link':
                RosUI.expressions.loadPage();
                break;
            case 'app-motors-link':
                RosUI.motors.loadPage();
                break;
            case 'app-animations-link':
                RosUI.animations.loadPage();
                break;
            case 'app-interactions-link':
                RosUI.interaction.loadPage();
                break;
        }
    }
};