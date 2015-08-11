define(['application', 'tpl!./templates/configuration.tpl', 'backgrid', './config/motor_grid', 'lib/api',
        'backbone_pageable', 'backgrid_select_all', 'backgrid_filter', 'backgrid_paginator'],
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
                    var Motor = Backbone.Model.extend({}),
                        MotorCollection = Backbone.PageableCollection.extend({
                            model: Motor,
                            url: "/motors/get/" + api.config.robot,
                            state: {
                                pageSize: 15
                            },
                            mode: "client", // page entirely on the client side
                            save: function (errorCallback) {
                                var data = [];
                                this.each(function (motor) {
                                    motor = motor.toJSON();
                                    data.push(motor);
                                });

                                $.ajax("/motors/update/" + api.config.robot, {
                                    data: JSON.stringify(data),
                                    type: 'POST',
                                    dataType: "json",
                                    success: function (data) {
                                        if (data.error) {
                                            alert(data.error);
                                        }
                                    },
                                    error: function () {
                                        if (typeof errorCallback == 'function')
                                            errorCallback();
                                    }
                                });
                            }
                        });

                    this.pageableMotors = new MotorCollection();

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

                    // Fetch some data
                    this.pageableMotors.fetch({
                        reset: true,
                        success: function (data) {
                            console.log(data);
                        }
                    });
                },
                add: function () {
                    this.pageableGrid.insertRow({name: "New"})
                },
                save: function () {
                    this.pageableMotors.save();
                },
                delete: function () {
                    var self = this;
                    $.each(this.pageableGrid.getSelectedModels(), function (i, e) {
                        self.pageableGrid.removeRow(e);
                    });
                }
            });
        });

        return App.module('Motors.Views.Configuration');
    });
