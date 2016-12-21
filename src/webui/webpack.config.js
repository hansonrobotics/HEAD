var webpack = require('webpack'),
    path = require('path'),
    autoprefixer = require('autoprefixer'),
    argv = require('yargs').options({
        d: {
            alias: 'dev',
            describe: 'Enable development environment',
            type: 'boolean',
            default: false
        }
    }).argv;

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
            loader: 'babel',
            query: {
                presets: ['es2015']
            }
        }, {
            test: /\.jpe?g|\.png|\.gif/,
            loader: "file-loader"
        }, {
            test: /\.woff(2)?(\?v=[0-9]\.[0-9]\.[0-9])?$/,
            loader: "url-loader?limit=10000&minetype=application/font-woff"
        }, {
            test: /\.(ttf|eot|svg|otf)(\?v=[0-9]\.[0-9]\.[0-9])?$/, loader: "file-loader"
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
        }, {
            test: /\.css$/,
            loader: 'style-loader!css-loader!postcss-loader'
        }, {
            test: /\.svg/,
            loader: 'svg-url-loader'
        }, {
            test: /\.yaml$/,
            loader: 'json!yaml'
        }]
    },
    output: {
        path: path.join(__dirname, './public/build'),
        filename: '[name].js',
        chunkFilename: '[id].js',
        publicPath: '/public/build/'
    },
    resolve: {
        alias: {
            marionette: 'backbone.marionette',
            scrollTo: 'jquery.scrollto',
            scrollbar: 'perfect-scrollbar/src/js/adaptor/jquery',
            selectareas: 'vendor/jquery.selectareas',
            typeahead: 'bootstrap-3-typeahead',
            multilevelpushmenu: 'vendor/jquery.multilevelpushmenu.min',
            'jquery-ui': 'jquery-ui-bundle/jquery-ui.js',
            emoticons: 'vendor/jquery.cssemoticons.min.js',
            'emoticons-css': 'css/vendor/jquery.cssemoticons.css',
            'backgrid-css': 'backgrid/lib/backgrid.css',
            'backgrid-paginator-css': 'backgrid-paginator/backgrid-paginator.css',
            'backgrid-select-all-css': 'backgrid-select-all/backgrid-select-all.css',
            'backgrid-filter-css': 'backgrid-filter/backgrid-filter.css',
            'backgrid-text-cell-css': 'backgrid-text-cell/backgrid-text-cell.css',
            'select2-css': 'select2/dist/css/select2.css',
            'scrollbar-css': 'perfect-scrollbar/dist/css/perfect-scrollbar.css',
            'font-awesome': 'css/vendor/font-awesome.css',
            'multilevelpushmenu-css': 'css/vendor/jquery.multilevelpushmenu.css',
            'selectareas-css': 'css/vendor/jquery.selectareas.css',
            'bootstrap-css': 'css/vendor/bootstrap.min.css',
            'jquery-ui-css': 'jquery-ui-bundle/jquery-ui.css',
            'nouislider-css': 'nouislider/distribute/nouislider.min.css'
        },
        extensions: ['', '.js', '.css', '.yaml'],
        root: [path.join(__dirname, './client')],
        modulesDirectories: ['node_modules', 'client']
    },
    plugins: [
        new webpack.ProvidePlugin({
            $: 'jquery',
            jQuery: 'jquery',
            "window.jQuery": "jquery",
            '_': 'lodash',
            d3: 'd3',
            eventemitter2: 'EventEmitter2',
            Marionette: 'marionette',
            Backbone: 'backbone'
        })
    ],
    postcss: [
        autoprefixer({
            browsers: ['last 2 versions']
        })
    ],
    debug: argv.dev,
    displayErrorDetails: argv.dev,
    outputPathinfo: argv.dev,
    devtool: argv.dev ? 'cheap-eval-source-map' : null
};
