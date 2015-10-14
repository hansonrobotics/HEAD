define(['application', 'tpl!./templates/log.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Logs = Marionette.ItemView.extend({
            initialize: function () {
                this.listenTo(this.model, "log:changed", this.addRows);
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
            tableBody: function(data) {
                var self = this
                var tbl_body = "";
                var warning = new RegExp('WARN|WARNING');
                var danger = new RegExp('ERROR|FATAL');
                var keys = ['name', 'levelname', 'asctime', 'message'];
                var error_count = 0, warning_count = 0;
                var re = new RegExp('/', 'g');
                var node = this.model.get('node').replace(re, '-');
                $.each(data, function() {
                    var tbl_row = "";
                    for(var key of keys) {
                        if (key=='message' && (this['extra'].length > 0)){
                            var id = 'extra_'+node+'_'+self.current_row
                            tbl_row += "<td>"+this[key]+"<a data-toggle='collapse' data-target='#"+id+"' class='label'>...</a><div id='"+id+"' class='collapse'>"
                            +this['extra'].join('<br>')
                            +"</div></td>";
                        }else{
                            tbl_row += "<td>"+this[key]+"</td>";
                        }

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
                    self.current_row += 1;
                })
                return {'error_count': error_count,
                        'warning_count': warning_count,
                        'body':tbl_body};
            },
            addRows: function(){
                console.log('add rows');
                var res = this.tableBody(this.model.new_logs);
                this.ui.body.prepend(res.body);
                this.error_count += res.error_count;
                this.warning_count += res.warning_count;
                this.row_count += this.model.new_logs.length;
                this.setCount();
            },
            setCount: function() {
                if (this.warning_count > 0) {
                    this.ui.warning_count.text(this.warning_count);
                }
                if (this.error_count > 0) {
                    this.ui.error_count.text(this.error_count);
                }
            },
            onRender: function () {
                var re = new RegExp('/', 'g');
                var node = this.model.get('node').replace(re, '-');
                var log = this.model.get('log');
                this.ui.a.attr("href", "#"+node);
                this.ui.collapse.attr("id", node);
                var res = this.tableBody(log);
                this.warning_count =  res.warning_count;
                this.error_count = res.error_count;
                this.row_count = log.length;
                this.ui.body.html(res.body)
                this.setCount();
            },
        });
    });

    return App.module('Monitor.Views').Logs;
});

