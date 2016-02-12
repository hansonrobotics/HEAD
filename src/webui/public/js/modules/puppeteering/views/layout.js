define(['application', 'marionette', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in', 'modules/performances/views/layout',
    'modules/interaction/views/interaction', 'jquery', './speech', 'modules/gestures/views/animation_mode',
    'modules/gestures/views/poses', 'modules/gestures/views/animations', 'scrollbar'],
    function (App, Marionette, template, FadeInRegion, TimelineEditorView, InteractionView, $, SpeechView,
              AnimationModeView, PosesView, AnimationsView) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                container: '.app-puppeteering-container',
                timeline: '.app-timeline-editor-region',
                controls: '.app-controls',
                leftColumn: '.app-left-column',
                rightColumn: '.app-right-column'
            },
            events: {},
            regions: {
                timeline: {
                    el: ".app-timeline-editor-region",
                    regionClass: FadeInRegion
                },
                animationMode: {
                    el: ".app-animation-mode-region",
                    regionClass: FadeInRegion
                },
                chat: {
                    el: ".app-chat-region",
                    regionClass: FadeInRegion
                },
                poses: {
                    el: '.app-pose-region',
                    regionClass: FadeInRegion
                },
                animations: {
                    el: '.app-animations-region',
                    regionClass: FadeInRegion
                }
            },
            onAttach: function () {
                var self = this;

                this.ui.leftColumn.perfectScrollbar();
                this.ui.rightColumn.perfectScrollbar();

                // left col
                this.posesView = new PosesView();
                this.animationModeView = new AnimationModeView();
                this.timelineEditorView = new TimelineEditorView({fluid: true});

                // right col
                this.chatView = new InteractionView({
                    faces_visible: true,
                    recognition_method: 'webspeech',
                    hide_method_select: true,
                    hide_noise: true
                });
                this.animationsView = new AnimationsView();

                this.getRegion('poses').show(this.posesView);
                this.getRegion('timeline').show(this.timelineEditorView);
                this.getRegion('animationMode').show(this.animationModeView);
                this.getRegion('chat').show(this.chatView);
                this.getRegion('animations').show(this.animationsView);

                var updateDimensions = function () {
                    if (self.isDestroyed)
                        $(window).off('resize', updateDimensions);
                    else
                        self.updateDimensions();
                };

                $(window).on('resize', updateDimensions).resize();
            },
            onDestroy: function () {
                this.timelineEditorView.destroy();
                this.chatView.destroy();
            },
            updateDimensions: function () {
                var height = App.LayoutInstance.getContentHeight();
                this.ui.leftColumn.height(height).perfectScrollbar('update');
                this.ui.rightColumn.height(height).perfectScrollbar('update');
                this.chatView.setHeight(height - this.ui.controls.outerHeight());
            }
        });
    });
