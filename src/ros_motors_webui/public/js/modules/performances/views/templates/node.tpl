<hr/>

<div class="row">
    <div class="col-sm-2">
        <label title="Emotion">Start time</label>

        <div class="input-group">
            <input type="text" class="app-node-start-time form-control" title="Offset" value="<%= start_time %>"/>

            <div class="input-group-addon">s</div>
        </div>
    </div>
    <% if (name == 'emotion' || name == 'interaction' || name == 'expression') { %>
        <div class="col-sm-2">
            <label>Duration</label>

            <div class="input-group">
                <input type="text" class="app-node-duration form-control" title="Duration" value="<%= duration %>"/>

                <div class="input-group-addon">s</div>
            </div>
        </div>
    <% } %>
    <% if (name == 'emotion') { %>
        <div class="col-sm-2">
            <label title="Emotion">Pose</label>
            <select class="app-emotion-select"></select>
        </div>
    <% } else if (name == 'gesture') { %>
        <div class="col-sm-2">
            <label>Speed <span class="app-speed-label pull-right label label-default"></span></label>

            <div class="app-speed-slider"></div>
        </div>
        <div class="col-sm-2">
            <label title="Animation">Animation</label>
            <select class="app-gesture-select"></select>
        </div>
    <% } %>
    <% if (name == 'expression') { %>
        <div class="col-sm-2">
            <label title="Expression">Expression</label>
            <select class="app-expression-select"></select>
        </div>
    <% } %>
    <% if (name == 'emotion' || name == 'gesture' || name == 'expression') { %>
        <div class="col-sm-3">
            <label title="Magnitude">Magnitude <span class="app-magnitude-label pull-right label label-default"></span></label>

            <div class="app-magnitide-slider"></div>
        </div>
    <% } %>

    <% if (name == 'speech') { %>
        <div class="col-sm-8">
            <label title="Text">Text</label>
            <textarea class="app-node-text form-control" title="Text"></textarea>
        </div>
    <% } else if (name == 'look_at' || name == 'gaze_at') { %>
        <div class="col-sm-3">
            <div class="app-crosshair"></div>
        </div>
    <% } %>

    <div class="col-sm-2">
        <button class="app-delete-node-button btn btn-default">Delete</button>
    </div>
</div>
<hr/>