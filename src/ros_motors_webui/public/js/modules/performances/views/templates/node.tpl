<hr/>

<div class="row">
    <div class="col-sm-3">
        <label title="Emotion">Offset</label>

        <div class="input-group">
            <input type="text" class="app-node-offset form-control" title="Offset" value="<%= offset %>"/>

            <div class="input-group-addon">s</div>
        </div>
    </div>

    <% if (name == 'emotion' || name == 'gesture') { %>
        <div class="col-sm-3">
            <label title="Emotion">Duration</label>

            <div class="input-group">
                <input type="text" class="app-node-duration form-control" title="Duration" value="<%= duration %>"/>

                <div class="input-group-addon">s</div>
            </div>
        </div>
        <% if (name == 'emotion') { %>
            <div class="col-sm-3">
                <label title="Emotion">Emotion</label>
                <select class="app-emotion-select"></select>
            </div>
        <% } else { %>
            <div class="col-sm-3">
                <label title="Gesture">Gesture</label>
                <select class="app-gesture-select"></select>
            </div>
        <% } %>

        <div class="col-sm-3">
            <label title="Magnitude">Magnitude</label>

            <div class="app-magnitide-slider"></div>
        </div>
    <% } %>

    <% if (name == 'speech') { %>
    <div class="col-sm-9">
        <label title="Text">Text</label>
        <textarea class="app-node-text form-control" title="Text"><%= text %></textarea>
    </div>
    <% } else if (name == 'look_at') { %>
    <div class="col-sm-9">
        <div class="app-crosshair"></div>
    </div>
    <% } %>
</div>
<hr/>