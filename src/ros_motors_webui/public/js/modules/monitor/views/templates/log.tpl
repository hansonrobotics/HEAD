<div class="panel panel-default">
    <div class="panel-heading" role="tab" id="heading">
        <h4 class="panel-title">
            <a class="collapsed" role="button" data-toggle="collapse" data-parent="#accordion"
               href="#collapse" aria-expanded="false" aria-controls="collapse">
                <%- node %>
                <span id="warning_count" class="pull-right label label-warning"></span>
                <span id="error_count" class="pull-right label label-danger"></span>
            </a>
        </h4>
    </div>
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

