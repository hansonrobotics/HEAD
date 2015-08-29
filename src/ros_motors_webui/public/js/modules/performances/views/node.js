define(['application', 'tpl!./templates/node.tpl', 'lib/api', 'jquery-ui', 'vendor/select2.min'],
    function (App, template, api) {
        App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
            Views.Node = Marionette.ItemView.extend({
                template: template,
                ui: {
                    magnitudeSlider: '.app-magnitide-slider',
                    emotionSelect: '.app-emotion-select',
                    offset: '.app-node-offset',
                    duration: '.app-node-duration'
                },
                events: {
                    'change @ui.duration': 'setDuration',
                    'change @ui.offset': 'setOffset'
                },
                modelEvents: {
                    "change": "modelChanged"
                },
                onRender: function () {
                    var self = this;

                    // init magnitude slider
                    this.ui.magnitudeSlider.slider({
                        range: 'min',
                        slide: function (e, ui) {
                            self.model.set('magnitude', ui.value)
                        }
                    });

                    // load emotions
                    api.getAvailableEmotionStates(function (emotions) {
                        _.each(emotions, function (emotion) {
                            self.ui.emotionSelect.append($('<option>').prop('value', emotion).html(emotion));
                        })
                    });

                    // init emotion select
                    this.ui.emotionSelect.select2();
                },
                modelChanged: function () {
                    switch (this.model.get('name')) {
                        case 'emotion':
                            break;
                    }
                },
                setDuration: function () {
                    this.model.set('duration', Number($(this.ui.duration).val()));
                },
                setOffset: function () {
                    this.model.set('offset', Number($(this.ui.offset).val()));
                }
            });
        });
    });
