$.fn.contextMenu = function(settings) {
    return this.each(function() {
        // Open context menu
        $(this).on("contextmenu", function(e) {
            // return native menu if pressing control
            if (e.ctrlKey) return

            // open menu
            let left = getMenuPosition(e.clientX, 'width', 'scrollLeft'),
                top = getMenuPosition(e.clientY, 'height', 'scrollTop'),
                $menu = $(settings.menuSelector).clone().appendTo('body')
                    .data("invokedOn", $(e.target))
                    .show()
                    .css({
                        position: "absolute",
                        left: left,
                        top: top
                    })
                    .addClass('app-context-menu')
                    .off('click')
                    .on('click', 'a', function(e) {
                        let $invokedOn = $menu.data("invokedOn")
                        let $selectedMenu = $(e.target)
                        $menu.remove()
                        settings.menuSelected.call(this, $invokedOn, $selectedMenu, {left: left, top: top})
                    })

            if (typeof settings.menuOpened === 'function')
                settings.menuOpened.call(this, $menu)

            $('body').trigger('click').on('click contextmenu', function removeMenu() {
                $menu.remove()
                $('body').off('click contextmenu', removeMenu)
            })

            return false
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