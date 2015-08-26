define(['application', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in', 'vendor/select2.min', 'jquery-ui'],
    function (App, template, FadeInRegion) {
        App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
            Views.Layout = Marionette.LayoutView.extend({
                template: template,
                regions: {
                    performances: {
                        el: '.app-performances-region',
                        regionClass: FadeInRegion
                    },
                    timeline: {
                        el: '.app-timeline-region',
                        regionClass: FadeInRegion
                    }
                },
                onRender: function () {
                    //var self = this;
                    //this.ui.settings.hide();

                    //$('.app-node', this.el).draggable({
                    //    helper: 'clone',
                    //    snap: '.app-timeline'
                    //});
                    //
                    //$(".app-timeline", this.el).droppable({
                    //    accept: ".app-node",
                    //    drop: function (event, ui) {
                    //        if ($(ui.helper).hasClass('app-copied')) {
                    //            $(this).append(ui.helper);
                    //        } else {
                    //            var el = $(ui.helper).clone(),
                    //                handle = $(document.createElement('span'))
                    //                    .addClass('glyphicon glyphicon-resize-horizontal app-node-resize-handle');
                    //
                    //            $(document).mouseup(function (e) {
                    //                var el = $('.app-timeline .app-node', self.el);
                    //
                    //                if (el.is(e.target || el.has(e.target).length === 0)) {
                    //                    self.ui.settings.slideDown();
                    //                } else {
                    //                    self.ui.settings.slideUp();
                    //                }
                    //            });
                    //
                    //            $(el).append(handle).addClass('app-copied').resizable({
                    //                grid: [0, 's'],
                    //                handles: {
                    //                    'e': handle
                    //                }
                    //            }).draggable({
                    //                snap: '.app-timeline'
                    //            }).resizable({
                    //                handles: 'e'
                    //            });
                    //
                    //            $(this).append(el);
                    //        }
                    //    }
                    //});

                    //$('.app-slider-demo', this.el).slider({value: 30});
                    //$('select', this.el).select2({value: 15});
                }
            });
        });
    });
