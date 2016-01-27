<div class="row">
    <div class="col-sm-12">
        <div class="form-inline">
            <div class="form-group">
                <label title="Emotion">Start time</label>

                <div class="input-group">
                    <input type="text" class="app-node-start-time form-control" title="Start time"
                           value="<%= start_time %>"/>

                    <div class="input-group-addon">s</div>
                </div>
            </div>

            <% if (name == 'emotion' || name == 'interaction' || name == 'expression') { %>
            <div class="form-group">
                <label>Duration</label>

                <div class="input-group">
                    <input type="text" class="app-node-duration form-control" title="Duration" value="<%= duration %>"/>

                    <div class="input-group-addon">s</div>
                </div>
            </div>
            <% } %>
            <% if (name == 'emotion') { %>
            <div class="form-group">
                <label title="Emotion">Pose</label>
                <select class="app-emotion-select"></select>
            </div>
            <% } else if (name == 'gesture') { %>
            <div class="form-group">
                <label>Speed <span class="app-speed-label pull-right label label-default"></span></label>

                <div class="app-speed-slider"></div>
            </div>
            <div class="form-group">
                <label title="Animation">Animation</label>
                <select class="app-gesture-select"></select>
            </div>
            <% } %>
            <% if (name == 'expression') { %>
            <div class="form-group">
                <label title="Expression">Expression</label>
                <select class="app-expression-select"></select>
            </div>
            <% } %>
            <% if (name == 'emotion' || name == 'gesture' || name == 'expression') { %>
            <div class="form-group">
                <label title="Magnitude">Magnitude <span
                        class="app-magnitude-label pull-right label label-default"></span></label>

                <div class="app-magnitide-slider"></div>
            </div>
            <% } %>
            <% if (name == 'speech') { %>
            <div class="form-group">
                <label title="Language">Language</label>
                <select class="app-lang-select">
                    <option value="en">English</option>
                    <option value="zh">Mandarin</option>
                </select>
            </div>
            <div class="form-group">
                <label title="Text">Text</label>
                <textarea class="app-node-text form-control" title="Text"></textarea>
            </div>
            <% } else if (name == 'look_at' || name == 'gaze_at') { %>
            <div class="form-group">
                <div class="app-crosshair"></div>
            </div>
            <% } %>

            <div class="node-setting-buttons form-group">
                <div class="btn-group" role="group" aria-label="...">
                    <button class="app-delete-node-button btn btn-danger"><i class="glyphicon glyphicon-trash"></i>
                        Delete
                    </button>
                    <button class="app-node-duration-indicator btn btn-default" disabled="disabled">0s</button>
                    <button class="app-node-frames-indicator btn btn-default" disabled="disabled">0</button>
                </div>
            </div>
        </div>
    </div>
</div>
