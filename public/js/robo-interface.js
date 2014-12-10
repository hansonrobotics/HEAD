//Utility function. Builds a wrapper function around the given function and
//makes sure it is not invoked in shorter time intervals than 'millis'. The
//last call is guaranteed to make it after the timer goes out.
var limitCallRate = function (millis, func) {
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
};

//Main Class
var RoboInterface = {

    $: $({}), //Event proxy. Events are triggered and bound to this object.

    //Keeps the incomming motor messages from flooding.


};