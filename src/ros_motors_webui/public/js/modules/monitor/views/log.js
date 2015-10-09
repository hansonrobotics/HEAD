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
                var odd_even = false;
                $.each(data, function() {
                    var tbl_row = "";
                    $.each(this, function(k , v) {
                        tbl_row += "<td>"+v+"</td>";
                    })
                    tbl_body += "<tr class=\""+( odd_even ? "odd" : "even")+"\">"+tbl_row+"</tr>";
                    odd_even = !odd_even;
                })
                return tbl_body;
            },
            onRender: function () {
                var re = new RegExp('/', 'g');
                var node = this.model.get('node').replace(re, '-');
                var log = this.model.get('log');
                tableBody = this.tableBody(log);
                console.log(tableBody);
                this.ui.label.text(log.length);
                this.ui.a.attr("href", "#"+node);
                this.ui.collapse.attr("id", node);
                this.ui.body.html(tableBody)
            },
        });
    });

    return App.module('Monitor.Views').Logs;
});

