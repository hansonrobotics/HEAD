<div id="app-dashboard" class="container">
    <div class="row">
        <div class="col-sm-3">
            <div class="control-stack">
                <div class="dashboard-intensity knobBlock">
                    <input value="17" type="text" class="app-intensity" disabled="disabled">
                </div>
            </div>
        </div>

        <div class="col-sm-9">
            <h3>Poses</h3>

            <div class="container-fluid">
                <% for (var i = 0; i < emotions.length / 3; i++) { %>
                <div class="row">
                    <% for (var j = 0; j < 3; j++) { %>
                    <div class="col-xs-4">
                        <% var emotion = emotions.at(3 * i + j) %>
                        <% if (!!emotion) { %>
                        <button class="app-emotion btn btn-default btn-lg"
                                data-emotion="<%= emotion.get('name') %>"><%= emotion.get('name') %></button>
                        <% } %>
                    </div>
                    <% } %>
                </div>
                <% } %>
            </div>
        </div>
    </div>

    <div class="row">
        <div class="col-sm-8">
            <h3>Performances</h3>

            <div class="container-fluid">
                <% for (var i = 0; i < performances.length / 3; i++) { %>
                <div class="row">
                    <% for (var j = 0; j < 3; j++) { %>
                    <div class="col-xs-4">
                        <% var performance = performances.at(3 * i + j) %>
                        <% if (!!performance) { %>
                        <button class="app-performance btn btn-default btn-lg"
                                data-performance="<%= performance.get('name') %>"><%= performance.get('name') %></button>
                        <% } %>
                    </div>
                    <% } %>
                </div>
                <% } %>
            </div>
        </div>

        <div class="col-sm-4">
            <h3>Say</h3>
            <textarea class="app-say-textarea app-text-input form-control"></textarea>

            <div class="say-button-group btn-group btn-group-justified">
                <div class="btn-group" role="group">
                    <button class="app-clear-button btn btn-default btn-lg">Clear</button>
                </div>
                <div class="dashboard-record-button-group btn-group" role="group">
                    <button class="app-say-button btn btn-info btn-lg"><i class="fa fa-volume-up fa-1x"></i>&nbsp;&nbsp;Say
                    </button>
                </div>
            </div>
        </div>
    </div>
</div>
