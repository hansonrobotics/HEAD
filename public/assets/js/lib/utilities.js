define(function () {
    return  {
        degToRad: function (val) {
            return val * Math.PI / 180;
        },
        radToDeg: function (val) {
            return val * 180 / Math.PI;
        },
        limitCallRate: function (millis, func) {
            var timeout = null;
            var last_args = null;

            function fire() {
                timeout = null;
                if (last_args != null) {
                    args = last_args;
                    last_args = null;
                    timeout = setTimeout(fire, millis);
                    func.apply(null, args); //Apply the last saved arguments
                }
            }

            return function () {
                last_args = arguments;
                if (timeout == null) {
                    fire();
                }
            };
        }
    };
});
