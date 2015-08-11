/*!
  backbone-japanese-input-handler-mixin
  https://github.com/hnakamur/backbone-japanese-input-handler-mixin

  Copyright (c) 2014 Hiroaki Nakamura
  Licensed under the MIT @license.
*/
(function (root, factory) {

  // CommonJS
  if (typeof exports == "object") {
    (function () {
      factory(module.exports, require("backbone"));
    }());
  }
  // Browser
  else {
    factory(root, root.Backbone);
  }

}(this, function (exports, Backbone) {

  "use strict";

  /**
     BackboneJapaneseInputHandlerMixin invokes search when being idle after
     texts are commited by japanese IME. If IME has uncommitted text, it does
     not invoke search even key events occurs.

     Usage example:

     BackboneJapaneseInputHandlerMixin.call(Backgrid.Extension.ClientSideFilter.prototype);
     var filter = new Backgrid.Extension.ClientSideFilter({
       collection: pageableTerritories,
       fields: ['name']
     });
  */
  exports.BackboneJapaneseInputHandlerMixin = (function() {
    var isFirefox = navigator.userAgent.indexOf('Firefox') != -1,
        _callHandler = function(event) {
          var val = this.$(event.target).val();
          if (val !== this._oldVal) {
            this.handlerMethod();
            this._oldVal = val;
          }
        },
        onFocus = function(event) {
          this._oldVal = this.$(event.target).val();
          this._readyToCallHandler = true;
        },
        onBlur = function(event) {
          this._callHandler(event);
        },
        onKeyup = function(event) {
          // When Enter is pressed, IME commits text.
          if (event.which === 13 || isFirefox) {
            this._readyToCallHandler = true;
          }

          // Set timer only when IME does not have uncommitted text.
          if (this._readyToCallHandler) {
            this._callHandler(event);
          }
        },
        onKeydown = function(event) {
          if (isFirefox) {
            // Firefox fires keydown for the first key, does not fire
            // keydown nor keyup event during IME has uncommitted text, 
            // fires keyup when IME commits or deletes all uncommitted text.
            this._readyToCallHandler = false;
          } else {
            // IE, Chrome and Safari fires events with event.which = 229 for
            // every keydown during IME has uncommitted text.
            // Note:
            // For IE, Chrome and Safari, I cannot detect the moment when
            // you delete all uncommitted text with pressing ESC or Backspace
            // appropriate times, so readyToCallHandler remains false at the
            // moment.
            //
            // However, it is not a problem. Because the text becomes same
            // as oldVal at the moment, we does not invoke handler anyway.
            //
            // Next time key is pressed and if it causes text to change,
            // keydown with event.which != 229 occurs, readyToCallHandler
            // becomes true and handler will be invoked.
            this._readyToCallHandler = (event.which !== 229);
          }
        };

    return function(config) {
      this.inputSelector =
        config && config.inputSelector || "input[type=search]";
      this.handlerMethod = config && config.handlerMethod || this.search;
      this._readyToCallHandler = true;
      this._oldVal = undefined;
      this._callHandler = _callHandler;
      this.onFocus = onFocus;
      this.onBlur = onBlur;
      this.onKeyup = onKeyup;
      this.onKeydown = onKeydown;
      this.events["focus " + this.inputSelector] = "onFocus";
      this.events["blur " + this.inputSelector] = "onBlur";
      this.events["keyup " + this.inputSelector] = "onKeyup";
      this.events["keydown " + this.inputSelector] = "onKeydown";
      return this;
    };
  })();
}));
