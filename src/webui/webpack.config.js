var webpack = require('webpack'),
    path = require('path'),
    autoprefixer = require('autoprefixer'),
    ExtractTextPlugin = require('extract-text-webpack-plugin');

module.exports = {
    // the base path which will be used to resolve entry points
    context: path.join(__dirname, 'client'),
    // the main entry point for our application's frontend JS
    entry: {
        app: ['./entry.js']
    },
    module: {
        loaders: [{
            test: /\.js$/,
            exclude: /node_modules/,
            loaders: ['babel-loader']
        }, {
            test: /\.woff|\.woff2|\.svg|.eot|\.ttf/,
            loader: 'url?prefix=font/&limit=10000'
        }, {
            test: /\.jpe?g|\.png|\.gif|\.svg/,
            loader: "file-loader"
        }, {
            test: /\.tpl/,
            loader: "underscore-template-loader",
            query: {
                withImports: false,
                engine: 'lodash',
                root: 'img',
                prependFilenameComment: __dirname
            }
        }, {
            test: require.resolve('jquery'),
            loader: 'expose?jQuery!expose?$'
        }, {
            test: require.resolve('eventemitter2'),
            loader: 'expose?EventEmitter2'
        }, {
            test: require.resolve('json-editor'),
            loader: 'exports?JSONEditor'
        }]
    },
    output: {
        path: path.join(__dirname, './public/build'),
        filename: '[name].js',
        publicPath: "/public/build/"
    },
    resolve: {
        alias: {
            marionette: 'backbone.marionette',
            scrollTo: 'jquery.scrollto',
            scrollbar: 'perfect-scrollbar/src/js/adaptor/jquery',
            backbone_pageable: 'backbone-pageable',
            backgrid_select_all: 'backgrid-select-all',
            backgrid_filter: 'backgrid-filter',
            backgrid_paginator: 'backgrid-paginator',
            selectareas: 'vendor/jquery.selectareas',
            backgrid_text_cell: 'backgrid-text-cell',
            typeahead: 'bootstrap-3-typeahead',
            json_editor: 'json-editor',
            multilevelpushmenu: 'vendor/jquery.multilevelpushmenu.min',
            'jquery-ui': 'jquery-ui-bundle/jquery-ui',
            emoticons: 'vendor/jquery.cssemoticons.min'
        },
        extensions: ['', '.js', '.png', '.gif', '.jpg', '.jpeg'],
        root: [path.join(__dirname, './client')],
        modulesDirectories: ['node_modules', 'client']
    },
    plugins: [
        new webpack.ProvidePlugin({
            $: 'jquery',
            jQuery: 'jquery',
            "window.jQuery":"jquery",
            '_': 'lodash',
            d3: 'd3',
            eventemitter2: 'EventEmitter2'
        }),
        new ExtractTextPlugin('[name].css')
    ],
    postcss: [
        autoprefixer({
            browsers: ['last 2 versions']
        })
    ],
    debug: true,
    displayErrorDetails: true,
    outputPathinfo: true,
    devtool: 'sourcemap'
};
