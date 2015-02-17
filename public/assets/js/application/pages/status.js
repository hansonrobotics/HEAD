define(['jquery', './../lib/api', 'roslib'], function ($, api, ROSLIB) {
    var status = {
        config: {
        },
        init: function () {
            function query() {
                $.ajax({
                    url: "system/status",
                    dataType: "json",

                    success: function(data) {
                        var indicators = "";

                        $.each(data, function() {
                            indicators += '<li><span class="glyphicon ' +
                            (this.success ? "glyphicon-ok" : "glyphicon-remove") +
                            '" aria-hidden="true"></span>' +
                            '<strong>' + this.label + '</strong> (' + this.cmd + ')' +
                            '</li>';
                        });

                        $("#app-status-indicators").html(indicators);
                    }
                });
            }

            query();
            setInterval(query, 5000);
        },
        loadPage: function () {
            var blinkMessage, treeMessage;

            api.blenderMode.disable();

            blinkMessage = new ROSLIB.Message({data: 'arthur:stop'});
            api.topics.cmdBllink.publish(blinkMessage);

            treeMessage = new ROSLIB.Message({data: 'btree_off'});
            api.topics.cmdTree.publish(treeMessage);

            api.setExpression("happy", 0);
            api.pointHead();
        }
    };

    return status;
});
