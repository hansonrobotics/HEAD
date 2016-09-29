define(['application', 'marionette', './templates/layout.tpl', 'lib/regions/fade_in', 'modules/performances/views/layout',
        'modules/interaction/views/interaction', 'modules/interaction/views/faces', 'jquery', '../../interaction/views/operator',
        'modules/gestures/views/animation_mode', 'modules/gestures/views/poses', 'modules/gestures/views/animations', './crosshairs',
        'scrollbar', 'scrollbar-css'],
    function (App, Marionette, template, FadeInRegion, TimelineEditorView, InteractionView, FacesView, $, OperatorView,
              AnimationModeView, PosesView, AnimationsView, CrosshairsView) {
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
                },
                faces: {
                    el: '.app-faces-region',
                    regionClass: FadeInRegion
                },
                crosshairs: {
                    el: '.app-cross-hairs-region',
                    regionClass: FadeInRegion
                },
                speech: {
                    el: '.app-speech-region',
                    regionClass: FadeInRegion
                }
            },
            onAttach: function () {
                var self = this;

                // left col
                this.posesView = new PosesView({config: {duration: {min: 1, max: 8}}});
                this.animationModeView = new AnimationModeView();
                this.timelineEditorView = new TimelineEditorView({fluid: true});

                // right col
                this.chatView = new InteractionView({
                    hide_faces: true,
                    recognition_method: 'webspeech',
                    hide_method_select: true,
                    hide_noise: true
                });

                this.crosshairsView = new CrosshairsView();
                this.facesView = new FacesView({always_visible: true, enableBTMode: true});
                this.animationsView = new AnimationsView({config: {speed: {min: 0.5, max: 2}}});
                this.speechView = new OperatorView({interactionView: this.chatView});

                this.getRegion('poses').show(this.posesView);
                this.getRegion('timeline').show(this.timelineEditorView);
                this.getRegion('animationMode').show(this.animationModeView);
                this.getRegion('chat').show(this.chatView);
                this.getRegion('crosshairs').show(this.crosshairsView);
                this.getRegion('faces').show(this.facesView);
                this.getRegion('animations').show(this.animationsView);
                this.getRegion('speech').show(this.speechView);

                var updateDimensions = function () {
                    if (self.isDestroyed)
                        $(window).off('resize', updateDimensions);
                    else
                        self.updateDimensions();
                };

                this.updateDimensions();
                $(window).resize(updateDimensions).resize();
                this.animationsView.collection.on('add', updateDimensions);
            },
            updateDimensions: function () {
                var contentHeight = App.LayoutInstance.getContentHeight(),
                    height = contentHeight;

                if (this.ui.leftColumn.offset().left == this.ui.rightColumn.offset().left) {
                    this.ui.leftColumn.height('auto').perfectScrollbar('destroy');
                    this.ui.rightColumn.height('auto').perfectScrollbar('destroy');
                } else {
                    this.ui.leftColumn.height(contentHeight).perfectScrollbar({
                        suppressScrollX: true,
                        wheelPropagation: true,
                        swipePropagation: true
                    });
                    this.ui.rightColumn.height(contentHeight).perfectScrollbar({
                        suppressScrollX: true,
                        wheelPropagation: true,
                        swipePropagation: true
                    });
                }

                this.chatView.setHeight(height - this.ui.controls.outerHeight());
            }
        });
    });
