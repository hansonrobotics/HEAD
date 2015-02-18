requirejs.config({
    baseUrl: 'assets/js',
    paths: {
        backbone: 'vendor/backbone',
        //'backbone.picky': 'vendor/backbone.picky',
        //'backbone.syphon': 'vendor/backbone.syphon',
        jquery: 'vendor/jquery.min',
        'jquery-ui': 'vendor/jquery-ui.min',
        json2: 'vendor/json2',
        //localstorage: 'vendor/backbone.localstorage',
        marionette: 'vendor/backbone.marionette',
        //spin: 'vendor/spin',
        //'spin.jquery': 'vendor/spin.jquery',
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
        //localstorage: ['backbone'],
        //'spin.jquery': ['spin', 'jquery'],
        tpl: ['text'],
        eventemitter: {
            exports: 'EventEmitter2'
        },
        roslib: {
            deps: ['eventemitter'],
            exports: 'ROSLIB'
        }
    }
});

require(['app', 'apps/motor/motor_app', 'main/init'], function (UI) {
    UI.start();
});
