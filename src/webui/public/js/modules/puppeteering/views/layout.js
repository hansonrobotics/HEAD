define(['application', 'marionette', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in', '../../performances/views/layout',
        '../../interaction/views/interaction', '../../interaction/entities/face_collection',
        '../../interaction/entities/message_collection', 'jquery', './controls', 'scrollbar'],
    function (App, Marionette, template, FadeInRegion, TimelineEditorView, InteractionView, FaceCollection,
              MessageCollection, $, ControlsView) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                container: '.app-puppeteering-container',
                timeline: '.app-timeline-editor-region',
                controls: '.app-robot-controls'
            },
            events: {},
            regions: {
                timeline: {
                    el: ".app-timeline-editor-region",
                    regionClass: FadeInRegion
                },
                controls: {
                    el: ".app-robot-controls",
                    regionClass: FadeInRegion
                },
                interaction: {
                    el: ".app-interaction-region",
                    regionClass: FadeInRegion
                }
            },
            onAttach: function () {
                var self = this;
                this.ui.timeline.perfectScrollbar();
                this.timelineEditorView = new TimelineEditorView({fluid: true});
                this.messageCollection = new MessageCollection();
                this.faces = new FaceCollection();
                this.interactionView = new InteractionView({
                    collection: this.messageCollection,
                    faceCollection: this.faces,
                    faces_visible: true,
                    recognition_method: 'webspeech',
                    hide_method_select: true,
                    hide_noise: true
                });
                this.controlsView = new ControlsView({interactionView: this.interactionView});

                this.getRegion('timeline').show(this.timelineEditorView);
                this.getRegion('controls').show(this.controlsView);
                this.getRegion('interaction').show(this.interactionView);

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
                this.interactionView.destroy();
            },
            updateDimensions: function () {
                var height = App.LayoutInstance.getContentHeight();
                this.ui.container.height(height);
                this.interactionView.setHeight(height - this.ui.controls.outerHeight());
            }
        });
    });
