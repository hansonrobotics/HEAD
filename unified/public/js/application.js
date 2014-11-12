$(document).ready(function() {
    var anchor   = window.location.hash,
        pageLink = $('.app-change-page[href="' + anchor + '"]');

    $('.app-change-page').click(function() {
        var pageEl = $(this).data('page');
        $('.app-change-page').removeClass('active');
        $(this).addClass('active');

        if ($('.app-page:visible').length == 0) {
            $(pageEl).fadeIn();
        } else {
            $('.app-page:visible').fadeOut(400, function() {
                $(pageEl).fadeIn();
            });
        }

        $('#app-title').html($(this).html());
        $('.navbar-toggle:visible:not(.collapsed)').click();
    });


    if ($(pageLink).length)
        $(pageLink).click();
    else
        $('.app-change-page.active').click();
});