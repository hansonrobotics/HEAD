define(['application', 'marionette', './templates/layout.tpl', 'lib/regions/fade_in', 'modules/performances/views/layout',
        'modules/interaction/views/interaction', 'modules/interaction/views/faces', 'jquery',
        '../../interaction/views/operator', 'modules/gestures/views/animation_mode', 'modules/gestures/views/poses',
        'modules/gestures/views/animations', './crosshairs', '../config/layout', 'scrollbar', 'scrollbar-css'],
    function(App, Marionette, template, FadeInRegion, TimelineEditorView, InteractionView, FacesView, $, OperatorView,
             AnimationModeView, PosesView, AnimationsView, CrosshairsView, config) {
        return Marionette.View.extend({
            template: template,
            ui: {
                container: '.app-puppeteering-container',
                timeline: '.app-timeline-editor-region',
                controls: '.app-controls',
                leftColumn: '.app-left-column',
                rightColumn: '.app-right-column',
                columns: '.app-puppeteering-column'
            },
            events: {},
            widgetRegions: {
                timeline: {
                    el: '[data-column="timelines"]',
                    regionClass: FadeInRegion
                },
                animationMode: {
                    el: '[data-column="animation_mode"]',
                    regionClass: FadeInRegion
                },
                chat: {
                    el: '[data-column="interaction"]',
                    regionClass: FadeInRegion
                },
                poses: {
                    el: '[data-column="poses"]',
                    regionClass: FadeInRegion
                },
                animations: {
                    el: '[data-column="animations"]',
                    regionClass: FadeInRegion
                },
                faces: {
                    el: '[data-column="face_select"]',
                    regionClass: FadeInRegion
                },
                crosshairs: {
                    el: '[data-column="crosshairs"]',
                    regionClass: FadeInRegion
                },
                speech: {
                    el: '[data-column="speech"]',
                    regionClass: FadeInRegion
                }
            },
            initialize: function() {
                this.parseConfig(config)
                this.updateDimensions = _.bind(this.updateDimensions, this)
            },
            serializeData: function() {
                return {
                    config: config,
                }
            },
            onAttach: function() {
                let self = this

                _.forOwn(this.widgetRegions, function(definition, name) {
                    self.addRegion(name, definition)
                })

                if (_.includes(this.widgets, 'poses')) {
                    this.posesView = new PosesView({config: {duration: {min: 1, max: 8}}})
                    this.getRegion('poses').show(this.posesView)
                }

                if (_.includes(this.widgets, 'timelines')) {
                    this.timelineEditorView = new TimelineEditorView({
                        fluid: true,
                        readonly: true,
                        autoplay: true,
                        nav: false,
                        disableSaving: true,
                        queueHeight: 300
                    })
                    this.getRegion('timeline').show(this.timelineEditorView)
                }

                if (_.includes(this.widgets, 'animation_mode')) {
                    this.animationModeView = new AnimationModeView()
                    this.getRegion('animationMode').show(this.animationModeView)
                }

                if (_.includes(this.widgets, 'crosshairs')) {
                    this.crosshairsView = new CrosshairsView()
                    this.getRegion('crosshairs').show(this.crosshairsView)
                }

                if (_.includes(this.widgets, 'face_select')) {
                    this.facesView = new FacesView({always_visible: true, enableBTMode: true})
                    this.getRegion('faces').show(this.facesView)
                }

                if (_.includes(this.widgets, 'animations')) {
                    this.animationsView = new AnimationsView({config: {speed: {min: 0.5, max: 2}}})
                    this.getRegion('animations').show(this.animationsView)
                    this.animationsView.collection.on('add', this.updateDimensions)
                }

                if (_.includes(this.widgets, 'interaction')) {
                    this.chatView = new InteractionView({
                        hide_faces: true,
                        recognition_method: 'webspeech',
                        hide_method_select: true,
                        hide_noise: true
                    })
                    this.getRegion('chat').show(this.chatView)
                    this.chatView.setHeight(500)
                }

                if (_.includes(this.widgets, 'speech')) {
                    this.speechView = new OperatorView({interactionView: this.chatView})
                    this.getRegion('speech').show(this.speechView)
                }

                this.updateDimensions()
                $(window).resize(this.updateDimensions)
            },
            onDestroy: function() {
                $(window).off('resize', this.updateDimensions)
            },
            updateDimensions: function() {
                let contentHeight = App.LayoutInstance.getContentHeight()

                if (this.ui.columns.length > 1 &&
                    this.ui.columns.first().offset().left === this.ui.columns.last().offset().left) {
                    this.ui.columns.height('auto').perfectScrollbar('destroy')
                } else {
                    this.ui.columns.height(contentHeight).perfectScrollbar({
                        suppressScrollX: true,
                        wheelPropagation: true,
                        swipePropagation: true
                    })
                }
            },
            parseConfig: function(config) {
                this.widgets = []
                for (let w of config) {
                    if (!('rows' in w)) w['rows'] = []

                    for (let row of w['rows']) {
                        if (!('cols' in row)) row['cols'] = {}

                        for (let col of row['cols']) {
                            if (col['type']) this.widgets.push(col['type'])
                        }
                    }
                }
            }
        })
    })
