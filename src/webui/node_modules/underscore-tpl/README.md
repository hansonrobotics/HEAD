# underscore-tpl

> `_.template` for objects

## Installation

### Bower

```shell
bower install underscore-tpl
```

### Npm

```shell
npm install underscore-tpl
```

## Usage

### API

```
_tpl( subject, values, [settings])
```

### Example

```js
var _tpl = require( 'underscore-tpl' );
var values = {
    foo    : "foo",
    badass : "Jules Winfield",
    qux    : {
        mofo : "mofo"
    }
};

var config = {
    baz          : "<%= qux.mofo %>",
    major         : {
        badass : "<%= badass %>"
    },
    "<%= foo %>" : "bar"
};

console.log( _tpl( config, values ) );
```
```shell
# output
{ baz: 'mofo', major: { badass: 'Jules Winfield' }, foo: 'bar' }
```

## Settings

`_tpl` accepts all [`_.template`](http://lodash.com/docs#templateSettings) settings and adds a few more:

* `ignoreKeys` [Boolean]: when `true` object keys will not be interpolated. (Default: `false`)
* `mustache|handlebars` [Boolean]: when `true` you can use mustache style tags `{{ }}` instead of ERB.

These can be set either in the `settings` parameter of _tpl, e.g.:

```js
var results = _tpl(subject, values, {
    ignorekeys: true
});
```

or globally:

```js
_tpl.templateSettings.ignoreKeys = true;
```


## License

Released under [MIT license](LICENSE-MIT)