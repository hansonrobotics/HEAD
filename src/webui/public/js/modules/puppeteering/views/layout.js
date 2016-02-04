define(['marionette', 'tpl!./templates/layout.tpl', 'lib/regions/fade_in', '../../performances/views/layout',
        '../../interaction/views/interaction', '../../interaction/entities/face_collection',
        '../../interaction/entities/message_collection', 'jquery', 'scrollbar'],
    function (Marionette, template, FadeInRegion, TimelineEditorView, InteractionView, FaceCollection,
              MessageCollection, $) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                container: '.app-puppeteering-container',
                timeline: '.app-timeline-editor-region'
            },
            events: {},
            regions: {
                timeline: {
                    el: ".app-timeline-editor-region",
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
                    faceCollection: this.faces
                });
                this.interactionView.on('resize', function (height) {
                    self.updateHeight(height);
                });

                this.getRegion('timeline').show(this.timelineEditorView);
                this.getRegion('interaction').show(this.interactionView);
            },
            onDestroy: function () {
                this.timelineEditorView.destroy();
                this.interactionView.destroy();
            },
            updateHeight: function (height) {
                console.log(height);
                this.ui.timeline.css('height', height).perfectScrollbar('update');
            }
        });
    });
