requirejs.config({
    baseUrl: '/public/js',
    paths: {
        jquery: 'vendor/jquery',
        'jquery-ui': 'vendor/jquery-ui',
        backbone: 'vendor/backbone',
        json2: 'vendor/json2',
        marionette: 'vendor/backbone.marionette',
        text: 'vendor/text',
        tpl: 'vendor/underscore-tpl',
        underscore: 'vendor/underscore',
        jsyaml: 'vendor/js-yaml.min',
        roslib: 'vendor/roslib',
        eventemitter: 'vendor/eventemitter2',
        bootstrap: 'vendor/bootstrap.min',
        touch_punch: 'vendor/jquery.ui.touch-punch.min',
        annyang: 'vendor/annyang',
        backgrid: 'vendor/backgrid-0.3.5',
        backgrid_filter: 'vendor/backgrid-filter',
        backgrid_paginator: 'vendor/backgrid-paginator',
        backgrid_select_all: 'vendor/backgrid-select-all',
        backgrid_text_cell: 'vendor/backgrid-text-cell',
        backbone_pageable: 'vendor/backbone-pageable',
        scrollbar: 'vendor/perfect-scrollbar.jquery.min',
        select2: 'vendor/select2.min',
        d3: 'vendor/d3.min',
        RecordRTC: 'vendor/RecordRTC.min',
        json_editor: 'vendor/jsoneditor.min',
        push_menu: 'vendor/jquery.multilevelpushmenu.min',
        bootbox: 'vendor/bootbox.min',
        emoticons: 'vendor/jquery.cssemoticons.min',
        typeahead: 'vendor/bootstrap3-typeahead',
        selectareas: 'vendor/jquery.selectareas',
        robot_config: '/robot_config',
        supermodel: 'vendor/supermodel.min',
        scrollTo: 'vendor/jquery.scrollTo.min'
    },
    shim: {
        scrollTo: ['jquery'],
        supermodel: {
            depends: ['backbone', 'underscore'],
            exports: 'Supermodel'
        },
        selectareas: ['jquery'],
        typeahead: ['jquery'],
        emoticons: ['jquery'],
        scrollbar: ['jquery'],
        select2: ['jquery', 'jquery-ui'],
        'jquery-ui': ['jquery'],
        bootstrap: ['jquery', 'jquery-ui'],
        bootbox: {
            deps: ['bootstrap', 'jquery'],
            exports: 'bootbox'
        },
        underscore: {
            exports: '_'
        },
        backbone: {
            deps: ['jquery', 'underscore', 'json2'],
            exports: 'Backbone'
        },
        annyang: {
            exports: 'annyang'
        },
        touch_punch: ['jquery-ui'],
        'backbone.picky': ['backbone'],
        'backbone.syphon': ['backbone'],
        marionette: {
            deps: ['backbone'],
            exports: 'Marionette'
        },
        tpl: ['text'],
        roslib: {
            deps: ['eventemitter'],
            exports: 'ROSLIB'
        },
        backgrid: {
            deps: ['backbone'],
            exports: 'Backgrid'
        },
        backbone_pageable: {
            deps: ['backbone']
        },
        backgrid_select_all: {
            deps: ['backgrid']
        },
        backgrid_filter: {
            deps: ['backgrid']
        },
        backgrid_paginator: {
            deps: ['backgrid']
        },
        backgrid_text_cell: {
            deps: ['backgrid', 'backbone']
        },
        d3: {
            exports: 'd3'
        },
        json_editor: {
            exports: 'JSONEditor'
        },
        push_menu: ['jquery']
    }
});

require(["eventemitter"], function (EventEmitter2) {
    window.EventEmitter2 = EventEmitter2;
});

define(['touch_punch', 'jquery', 'jquery-ui', 'bootstrap', 'application']);
