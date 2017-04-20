<div class="log-panel panel panel-default">
    <a class="collapsed" role="button" data-toggle="collapse" data-parent="#accordion"
               href="#collapse" aria-expanded="false" aria-controls="collapse">
    <div class="panel-heading" role="tab" id="heading">
        <h2 class="panel-title">
                <%- node %>
                <div class="pull-right">
                    <span id="warning_count" class="label label-warning"></span>
                    <span id="error_count" class="label label-danger"></span>
                </div>
        </h2>
    </div>
    </a>
    <div id="collapse" class="panel-collapse collapse" role="tabpanel" aria-labelledby="heading">
        <div class="panel-body">
            <table class="log-table table">
                <thead>
                    <tr>
                        <th>Name</th>
                        <th>Level</th>
                        <th>Time</th>
                        <th>Message</th>
                    </tr>
                </thead>
                <tbody class="table-body"></tbody>
            </table>
        </div>
    </div>
</div>

