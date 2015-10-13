define(['application', 'tpl!./templates/log.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Logs = Marionette.ItemView.extend({
            initialize: function () {
                this.listenTo(this.model, "log:changed", this.addRows);
                console.log("added lkistener")
            },
            template: template,
            error_count: 0,
            warning_count: 0,
            ui: {
                error_count: '#error_count',
                warning_count: '#warning_count',
                title: '.title',
                body: '.table-body ',
                collapse: '.panel-collapse',
                a: '.collapsed'
            },
            tableBody: function(data) {
                var tbl_body = "";
                var warning = new RegExp('WARN|WARNING');
                var danger = new RegExp('ERROR|FATAL');
                var keys = ['name', 'levelname', 'asctime', 'message'];
                var error_count = 0, warning_count = 0;
                $.each(data, function() {
                    var tbl_row = "";
                    for(var key of keys) {
                        tbl_row += "<td>"+this[key]+"</td>";
                    }
                    if (warning.test(this['levelname'])) {
                        tbl_body += "<tr class=\"text-warning\">"+tbl_row+"</tr>";
                        warning_count++;
                    } else if (danger.test(this['levelname'])) {
                        tbl_body += "<tr class=\"text-danger\">"+tbl_row+"</tr>";
                        error_count++;
                    } else {
                        tbl_body += "<tr>"+tbl_row+"</tr>";
                    }
                })
                return {'error_count': error_count,
                        'warning_count': warning_count,
                        'body':tbl_body};
            },
            addRows: function(){

                var html = this.tableBody(this.model.new_logs);
                this.ui.body.prepend(html.body);
                this.error_count += html.error_count;
                this.warning_count += html.warning_count;
                this.ui.error_count.text(this.error_count);
                this.ui.warning_count.text(this.warning_count);
            },
            onRender: function () {
                var re = new RegExp('/', 'g');
                var node = this.model.get('node').replace(re, '-');
                var log = this.model.get('log');
                this.ui.a.attr("href", "#"+node);
                this.ui.collapse.attr("id", node);
                var res = this.tableBody(log);
                if (res.warning_count > 0) {
                    this.warning_count =  res.warning_count;
                    this.ui.warning_count.text(this.warning_count);
                }
                if (res.error_count> 0) {
                    this.error_count = res.error_count;
                    this.ui.error_count.text(this.error_count);
                }
                this.ui.body.html(res.body)
            },
        });
    });

    return App.module('Monitor.Views').Logs;
});

