define(['application', './public/controller', './admin/controller'],
    function (App, PublicController, AdminController) {
        App.module('Expressions', function (Expressions, app, Backbone, Marionette, $, _) {
            Expressions.PublicRouter = Marionette.AppRouter.extend({
                'appRoutes': {
                    '': 'index',
                    'expressions': 'index'
                }
            });

            Expressions.AdminRouter = Marionette.AppRouter.extend({
                'appRoutes': {
                    'admin': 'index',
                    'admin/expressions': 'index'
                }
            });

            Expressions.on('start', function () {
                new Expressions.PublicRouter({controller: PublicController});
                new Expressions.AdminRouter({controller: AdminController});
            });
        });
    });
