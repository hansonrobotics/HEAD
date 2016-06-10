define(['application', 'marionette', 'tpl!./templates/attention_regions.tpl', '../entities/attention_regions',
        'underscore', 'jquery', 'selectareas', 'select2'],
    function (App, Marionette, template, AttentionRegions, _, $) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                container: '.app-selectareas-container',
                areaselect: '.app-selectareas',
                save: '.app-save-areas',
                clear: '.app-clear-areas',
                typeSelect: 'select.app-region-type'
            },
            events: {
                'click @ui.clear': 'clear',
                'click @ui.save': 'save',
                'change @ui.typeSelect': 'selectType'
            },
            config: {
                width: 200,
                height: 100,
                convertRatio: 0.01,
                regions: {
                    main: 'Main Person',
                    audience: 'Audience',
                    specific: 'Specific'
                }
            },
            onAttach: function () {
                var self = this;

                this.enableTypeSelect();
                if (this.ui.areaselect.get(0).naturalWidth) {
                    self.enableAreaSelect();
                    self.fetchAreas();
                } else {
                    this.ui.areaselect.on('load', function () {
                        self.enableAreaSelect();
                        self.fetchAreas();
                    });
                }
            },
            enableTypeSelect: function () {
                var self = this;
                _.each(this.config.regions, function (name, key) {
                    self.ui.typeSelect.append($('<option>').attr('value', key).html(name));
                });
                this.ui.typeSelect.select2();
            },
            enableAreaSelect: function () {
                var self = this;

                this.width = this.ui.container.innerWidth();
                this.ui.areaselect.selectAreas({
                    allowSelect: false,
                    allowDelete: false,
                    width: this.width
                }).on('changed', function (event, id) {
                    var model = self.collection.get(id),
                        area = self.areaToJSON(_.findWhere(self.ui.areaselect.selectAreas('relativeAreas'), {id: id}));

                    if (model && area)
                        model.set(area);
                    else if (model)
                        self.collection.remove(model);
                    else if (area)
                        self.collection.add(area);

                    self.setActiveRegion(id);
                });
            },
            fetchAreas: function () {
                var self = this;
                if (!this.collection) {
                    this.collection = new AttentionRegions();
                    this.collection.fetch({
                        success: function () {
                            self.updateAreas();
                        }
                    });
                }
            },
            updateAreas: function () {
                var self = this,
                    models = [];

                _.each(this.config.regions, function (name, key) {
                    var model = self.collection.findWhere({type: key});
                    if (!model)
                        model = {
                            id: models.length,
                            width: 0.5,
                            height: 0.5,
                            x: -1 + 0.6 * models.length,
                            y: 0.5,
                            type: key
                        };
                    else
                        model.set('id', models.length);
                    models.push(model);
                });

                this.ui.areaselect.selectAreas('reset');
                this.collection.reset(models);
                this.collection.each(function (area) {
                    self.ui.areaselect.selectAreas('add', self.jsonToArea(area.toJSON()));
                });
            },
            save: function () {
                var self = this;
                this.collection.save({
                    success: function () {
                        App.Utilities.showPopover(self.ui.save, 'Saved', 'left');
                    },
                    error: function () {
                        App.Utilities.showPopover(self.ui.save, 'Error saving', 'left');
                    }
                });
            },
            clear: function () {
                this.ui.clear.blur();
                this.ui.areaselect.selectAreas('reset');
                this.collection.reset();
            },
            lastId: null,
            setActiveRegion: function (id) {
                var region = this.collection.get(id);

                if (region) {
                    this.lastId = id;
                    $('.select2-container.app-region-type', this.el).fadeIn();
                    this.ui.typeSelect.val(region.get('type')).trigger('change');
                } else {
                    this.lastId = null;
                    $('.select2-container.app-region-type', this.el).fadeOut();
                }
            },
            selectType: function () {
                var type = this.ui.typeSelect.val(),
                    model = this.collection.findWhere({type: type});

                if (model) this.ui.areaselect.selectAreas('focus', model.id);
            },
            updateType: function () {
                if (this.lastId != null) {
                    var region = this.collection.get(this.lastId);
                    if (region)
                        region.set('type', this.ui.typeSelect.val());
                }
            },
            areaToJSON: function (area) {
                if (area) {
                    var halfWidth = this.config.width / 2,
                        halfHeight = this.config.height / 2;

                    area = {
                        id: area.id,
                        width: (area.width) * this.config.convertRatio,
                        height: (area.height) * this.config.convertRatio,
                        x: (area.x - halfWidth) * this.config.convertRatio,
                        y: (this.config.height - area.y - halfHeight) * this.config.convertRatio
                    };
                }

                return area;
            },
            jsonToArea: function (json) {
                if (json) {
                    var halfWidth = this.config.width / 2,
                        halfHeight = this.config.height / 2,
                        ratio = this.width / this.config.width;

                    json = {
                        id: json.id,
                        width: (json.width / this.config.convertRatio) * ratio,
                        height: (json.height / this.config.convertRatio) * ratio,
                        x: (json.x / this.config.convertRatio + halfWidth) * ratio,
                        y: (this.config.height - (json.y / this.config.convertRatio + halfHeight)) * ratio
                    };
                }

                return json;
            }
        });
    });
