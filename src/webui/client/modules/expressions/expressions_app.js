define(['application', './public/controller', './admin/controller'],
    function (App, PublicController, AdminController) {
        var PublicRouter = Marionette.AppRouter.extend({
            'appRoutes': {
                'expressions': 'index'
            }
        });

        var AdminRouter = Marionette.AppRouter.extend({
            'appRoutes': {
                'admin/expressions': 'index'
            }
        });

        new PublicRouter({controller: PublicController});
        new AdminRouter({controller: AdminController});
    })
