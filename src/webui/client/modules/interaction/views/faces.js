define(['marionette', './templates/faces.tpl', '../entities/face_collection', 'jquery'],
    function (Marionette, template, FaceCollection, $) {
        return Marionette.View.extend({
            template: template,
            enableBTMode: false,
            ui: {
                expandFacesButton: '.app-expand-faces-button',
                faceThumbnails: '.app-face-thumbnails',
                faceContainer: '.app-select-person-container',
                faceCollapse: '.app-face-container'
            },
            initialize: function (options) {
                if (!options.collection)
                    this.collection = new FaceCollection();
            },
            serializeData: function () {
                return {
                    faces: this.collection
                };
            },
            onRender: function () {
                var self = this;
                this.listenTo(this.collection, 'change', this.updateFaces);
                this.collection.subscribe();
                if (this.options.always_visible)
                    this.setFaceVisibility(true);
                if (this.options.enableBTMode)
                    this.enableBTMode = true;
                this.updateFaces();

                // update chat margins on face collapse show/hide
                this.ui.faceCollapse.on('shown.bs.collapse hidden.bs.collapse', function () {
                    self.trigger('toggle');
                });
            },
            onDestroy: function () {
                this.collection.unsubscribe();
            },
            setFaceVisibility: function (visible) {
                if (visible) {
                    this.ui.faceCollapse.addClass('in');
                    this.ui.expandFacesButton.hide();
                } else {
                    this.ui.faceCollapse.removeClass('in');
                    this.ui.expandFacesButton.fadeIn();
                }
            },
            updateFaces: function () {
                var self = this,
                    currentTime = new Date().getTime();

                // remove lost faces older than 3 seconds
                $('img', this.ui.faceThumbnails).each(function (i, img) {
                    var id = parseInt($(img).attr('title'));

                    if (!self.collection.findWhere({id: id}) && (currentTime - $(img).data('time-added')) > 3000) {
                        $(img).remove();

                        if (self.collection.getLookAtFaceId() == id)
                            self.ui.faceCollapse.collapse('show');
                    }
                });

                this.collection.each(function (face) {
                    var img = $('img[title="' + face.get('id') + '"]', self.ui.faceThumbnails),
                    // update thumbnail every 3 seconds, update time added
                        thumbnailUrl = face.getThumbnailUrl() + '?' + parseInt(currentTime / 3000);

                    // if image already shown
                    if (img.length > 0) {
                        $(img).prop({
                            src: thumbnailUrl
                        }).data('time-added', currentTime);
                    } else {
                        // create new thumbnail
                        var setActiveThumbnail = function (el) {
                                $('img', self.ui.faceThumbnails).removeClass('active');
                                $(el).addClass('active');
                            },
                            el = $('<img>').prop({
                                src: thumbnailUrl,
                                title: face.get('id'),
                                'class': 'face-thumbnail thumbnail',
                                width: 100,
                                height: 100
                            }).data('time-added', currentTime).click(function () {
                                if (self.enableBTMode)
                                    self.collection.enableBTMode();
                                self.collection.setLookAtFaceId(face.get('id'));
                                setActiveThumbnail(this);
                            });

                        if (self.collection.getLookAtFaceId() == face.get('id'))
                            setActiveThumbnail(el);

                        self.ui.faceThumbnails.append(el);
                    }
                });

                if (!this.options.always_visible) {
                    if ($('img', this.ui.faceThumbnails).length == 0 && this.collection.isEmpty()) {
                        if (!this.facesEmpty) {
                            this.facesEmpty = true;
                            this.ui.faceContainer.slideUp();
                        }
                    } else if (typeof this.facesEmpty == 'undefined' || this.facesEmpty) {
                        this.facesEmpty = false;

                        this.ui.faceCollapse.removeClass('in');
                        this.ui.faceContainer.slideDown();
                    }
                }
            }
        });
    });