define(['application', './templates/log.tpl'], function (App, template) {
    App.module('Start.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Logs = Marionette.ItemView.extend({
            initialize: function () {
                this.listenTo(this.model, "log:changed", this.addRows);
                this.node = this.model.get('node').replace(new RegExp('/', 'g'), '-');
            },
            template: template,
            error_count: 0,
            warning_count: 0,
            row_count: 0,
            current_row: 0,
            ui: {
                error_count: '#error_count',
                warning_count: '#warning_count',
                title: '.title',
                body: '.table-body ',
                collapse: '.panel-collapse',
                a: '.collapsed'
            },
            tableBody: function (data) {
                var self = this
                var tbl_body = "";
                var warning = new RegExp('WARN|WARNING', 'i');
                var danger = new RegExp('ERROR|FATAL', 'i');
                var keys = ['name', 'levelname', 'asctime', 'message'];
                var error_count = 0, warning_count = 0;
                $.each(data, function () {
                    var row_id = self.node + '_' + self.current_row
                    var tbl_row = "";
                    keys.forEach(function (key) {
                        if (key == 'message' && (this['extra'].length > 0)) {
                            var id = 'extra_' + row_id;
                            tbl_row += "<td>" + this[key] + "<a data-toggle='collapse' data-target='#" + id + "' class='label'>...</a><div id='" + id + "' class='collapse'>"
                                + this['extra'].join('<br>')
                                + "</div></td>";
                        } else {
                            tbl_row += "<td>" + this[key] + "</td>";
                        }

                    });
                    if (warning.test(this['levelname'])) {
                        tbl_body += "<tr class=\"text-warning\">" + tbl_row + "</tr>";
                        warning_count++;
                    } else if (danger.test(this['levelname'])) {
                        tbl_body += "<tr class=\"text-danger\">" + tbl_row + "</tr>";
                        error_count++;
                    } else {
                        tbl_body += "<tr>" + tbl_row + "</tr>";
                    }
                    self.current_row += 1;
                })
                return {
                    'error_count': error_count,
                    'warning_count': warning_count,
                    'body': tbl_body
                };
            },
            addRows: function () {
                console.log('add rows');
                var res = this.tableBody(this.model.new_logs);
                this.ui.body.prepend(res.body);
                this.error_count += res.error_count;
                this.warning_count += res.warning_count;
                this.row_count += this.model.new_logs.length;
                this.adjustRows();
                this.setCount();
            },
            adjustRows: function () {
                var log_to_show = this.model.get('log_to_show');
                var log_to_remove = [];
                $('tr', 'tbody#tbl_body_' + this.node).each(function (index) {
                    if (index >= log_to_show) {
                        log_to_remove.push(this);
                    }
                });
                $.each(log_to_remove, function () {
                    this.remove();
                });

                if (log_to_remove.length > 0) {
                    var error_count = 0, warning_count = 0;
                    this.warning_count = $('tr.text-warning', 'tbody#tbl_body_' + this.node).length;
                    this.error_count = $('tr.text-danger', 'tbody#tbl_body_' + this.node).length;
                    this.setCount();
                }
            },
            setCount: function () {
                this.ui.warning_count.text(this.warning_count);
                this.ui.error_count.text(this.error_count);
            },
            onRender: function () {
                var log = this.model.get('log');
                this.ui.a.attr("href", "#" + this.node);
                this.ui.collapse.attr("id", this.node);
                var res = this.tableBody(log);
                this.warning_count = res.warning_count;
                this.error_count = res.error_count;
                this.row_count = log.length;
                this.ui.body.html(res.body)
                this.ui.body.attr("id", "tbl_body_" + this.node);
                this.adjustRows();
                this.setCount();
            },
        });
    });

    return App.module('Start.Views').Logs;
});

