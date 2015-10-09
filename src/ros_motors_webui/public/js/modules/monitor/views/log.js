define(['application', 'tpl!./templates/log.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Logs = Marionette.ItemView.extend({
            template: template,
            ui: {
                label: '.pull-right',
                title: '.title',
                body: '.table-body ',
                collapse: '.panel-collapse',
                a: '.collapsed',
            },
            tableBody: function(data) {
                var tbl_body = "";
                var danger = new RegExp('WARN|WARNING|ERROR|FATAL');
                var keys = ['name', 'levelname', 'asctime', 'message'];
                var danger_count = 0;
                $.each(data, function() {
                    var tbl_row = "";
                    for(var key of keys) {
                        tbl_row += "<td>"+this[key]+"</td>";
                    }
                    if (danger.test(this['levelname'])) {
                        tbl_body += "<tr class=\"text-danger\">"+tbl_row+"</tr>";
                        danger_count++;
                    } else {
                        tbl_body += "<tr>"+tbl_row+"</tr>";
                    }
                })
                return {'count': danger_count, 'body':tbl_body};
            },
            onRender: function () {
                var re = new RegExp('/', 'g');
                var node = this.model.get('node').replace(re, '-');
                var log = this.model.get('log');
                this.ui.a.attr("href", "#"+node);
                this.ui.collapse.attr("id", node);
                var res = this.tableBody(log);
                if (res.count > 0) {
                    this.ui.label.text(res.count);
                }
                this.ui.body.html(res.body)
            },
        });
    });

    return App.module('Monitor.Views').Logs;
});

