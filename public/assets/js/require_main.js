requirejs.config({
    baseUrl: '/assets/js',
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
        'touch-punch': 'vendor/jquery.ui.touch-punch.min'
    },
    shim: {
        'jquery-ui': ['jquery'],
        bootstrap: ['jquery'],
        underscore: {
            exports: '_'
        },
        backbone: {
            deps: ['jquery', 'underscore', 'json2'],
            exports: 'Backbone'
        },
        'touch-punch': ['jquery-ui'],
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
        }
    }
});

require(["eventemitter"], function (EventEmitter2) {
    window.EventEmitter2 = EventEmitter2;
});

define(['jquery', 'bootstrap', 'application']);
