define(['application', 'tpl!./templates/configuration.tpl', 'backgrid', './config/motor_grid', 'lib/api',
        'entities/motor', 'backbone_pageable', 'backgrid_select_all', 'backgrid_filter', 'backgrid_paginator', 'scrollbar'],
    function (App, template, Backgrid, columns, api) {
        App.module("Motors.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Configuration = Marionette.LayoutView.extend({
                template: template,
                ui: {
                    grid: ".app-motor-grid",
                    addButton: '.app-add-button',
                    saveButton: '.app-save-button',
                    deleteButton: '.app-delete-button'
                },
                events: {
                    'click @ui.addButton': 'add',
                    'click @ui.saveButton': 'save',
                    'click @ui.deleteButton': 'delete'
                },
                regions: {
                    filter: '.app-filter'
                },
                onRender: function () {
                    var self = this,
                        PageableCollection = Backbone.PageableCollection.extend({
                            mode: 'client',
                            state: {
                                pageSize: 65
                            }
                        });

                    this.motorsCollection = new App.Entities.MotorCollection();
                    this.pageableMotors = new PageableCollection();
                    this.motorsCollection.fetchFromFile(function () {
                        self.pageableMotors.add(self.motorsCollection.models);
                    });

                    // Set up a grid to use the pageable collection
                    this.pageableGrid = new Backgrid.Grid({
                        columns: columns,
                        collection: this.pageableMotors
                    });

                    // Render the grid
                    this.ui.grid.append(this.pageableGrid.render().el);

                    // Initialize the paginator
                    var paginator = new Backgrid.Extension.Paginator({
                        collection: this.pageableMotors
                    });

                    // Render the paginator
                    this.ui.grid.after(paginator.render().el);

                    // Initialize a client-side filter to filter on the client
                    // mode pageable collection's cache.
                    var filter = new Backgrid.Extension.ClientSideFilter({
                        collection: this.pageableMotors,
                        fields: ['name']
                    });

                    // Render the filter
                    this.getRegion('filter').show(filter);
                },
                add: function () {
                    this.pageableGrid.insertRow({name: "New"})
                },
                save: function () {
                    var el = this.ui.saveButton;
                    this.motorsCollection.reset(this.pageableMotors.toJSON());
                    this.motorsCollection.sync(function () {
                        App.Utilities.showPopover(el, 'Motors saved');
                    }, function (error) {
                        App.Utilities.showPopover(el, 'Error saving motors: ' + error);
                    });
                },
                delete: function () {
                    var self = this;
                    $.each(this.pageableGrid.getSelectedModels(), function (i, e) {
                        // remove from collection
                        self.motorsCollection.remove(e);

                        // remove from the grid
                        self.pageableGrid.removeRow(e);
                    });
                }
            });
        });

        return App.module('Motors.Views.Configuration');
    });
