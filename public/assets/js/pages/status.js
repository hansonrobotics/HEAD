define(['jquery', 'lib/api', 'roslib'], function ($, api, ROSLIB) {
    var status = {
        init: function () {
            function query() {
                $.ajax({
                    url: "system/status",
                    dataType: "json",

                    success: function (data) {
                        var indicators = "";

                        $.each(data, function () {
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
            api.blenderMode.disable();
            api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
            api.setExpression("Neutral", 0);
        }
    };

    return status;
});
