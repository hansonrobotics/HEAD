$.fn.contextMenu = function(settings) {
    return this.each(function() {
        // Open context menu
        $(this).on("contextmenu", function(e) {
            // return native menu if pressing control
            if (e.ctrlKey) return

            // open menu
            let left = getMenuPosition(e.clientX, 'width', 'scrollLeft'),
                top = getMenuPosition(e.clientY, 'height', 'scrollTop'),
                $menu = $(settings.menuSelector)
                    .data("invokedOn", $(e.target))
                    .show()
                    .css({
                        position: "absolute",
                        left: left,
                        top: top
                    })
                    .off('click')
                    .on('click', 'a', function(e) {
                        $menu.hide()

                        let $invokedOn = $menu.data("invokedOn")
                        let $selectedMenu = $(e.target)

                        settings.menuSelected.call(this, $invokedOn, $selectedMenu, {left: left, top: top})
                    })

            return false
        })

        //make sure menu closes on any click
        $('body').click(function() {
            $(settings.menuSelector).hide()
        })
    })

    function getMenuPosition(mouse, direction, scrollDir) {
        let win = $(window)[direction](),
            scroll = $(window)[scrollDir](),
            menu = $(settings.menuSelector)[direction](),
            position = mouse + scroll

        // opening menu would pass the side of the page
        if (mouse + menu > win && menu < mouse)
            position -= menu

        return position
    }

}