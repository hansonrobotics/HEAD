requirejs.config({
    baseUrl: 'assets/js',
    // disables caching for development
    urlArgs: "bust=" + (new Date()).getTime(),
    paths: {
        backbone: 'vendor/backbone',
        jquery: 'vendor/jquery.min',
        'jquery-ui': 'vendor/jquery-ui.min',
        json2: 'vendor/json2',
        marionette: 'vendor/backbone.marionette',
        text: 'vendor/text',
        tpl: 'vendor/underscore-tpl',
        underscore: 'vendor/underscore',
        jsyaml: 'vendor/js-yaml.min',
        roslib: 'vendor/roslib',
        eventemitter: 'vendor/eventemitter2',
        bootstrap: 'vendor/bootstrap.min'
    },
    shim: {
        underscore: {
            exports: '_'
        },
        backbone: {
            deps: ['jquery', 'underscore', 'json2'],
            exports: 'Backbone'
        },
        'backbone.picky': ['backbone'],
        'backbone.syphon': ['backbone'],
        marionette: {
            deps: ['backbone'],
            exports: 'Marionette'
        },
        'jquery-ui': ['jquery'],
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

define(['bootstrap', 'application']);
