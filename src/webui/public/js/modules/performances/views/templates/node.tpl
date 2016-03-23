<div class="row">
    <div class="col-sm-12">
        <div class="form-inline">
            <div class="form-group">
                <label>Start time</label>

                <div class="input-group">
                    <input type="text" class="app-node-start-time form-control" title="Start time"
                           value="<%= start_time %>"/>

                    <div class="input-group-addon">s</div>
                </div>
            </div>

            <% if (name == 'emotion' || name == 'interaction' || name == 'listening' || name == 'expression' || name == 'chat_pause' || name == 'soma') { %>
            <div class="form-group">
                <label>Duration</label>

                <div class="input-group">
                    <input type="text" class="app-node-duration form-control" title="Duration" value="<%= duration %>"/>

                    <div class="input-group-addon">s</div>
                </div>
            </div>
            <% } %>

            <% if (name == 'chat_pause') { %>
            <div class="form-group">
                <label>Message</label>

                <div class="input-group">
                    <input type="text" class="app-node-message-input form-control" title="Message" value="<%= message %>"/>
                </div>
            </div>
            <% } %>

            <% if (name == 'emotion') { %>
            <div class="form-group">
                <label>Pose</label>
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
            <% if (name == 'kfanimation') { %>
                <div class="form-group">
                    <label title="Animation">Animation</label>
                    <select class="app-kfanimation-select"></select>
                </div>
                <div class="form-group">
                    <label title="FPS">FPS <span
                            class="app-fps-label pull-right label label-default"></span></label>
                    <div class="app-fps-slider"></div>
                </div>
                <div class="form-group">
                    <label title="Arms">Blender Enabled</label>
                    <select class="app-kfmode-select">
                        <option value="on">Yes</option>
                        <option value="head">No</option>
                    </select>
                </div>
            <% } %>
            <% if (name == 'expression') { %>
            <div class="form-group">
                <label title="Expression">Expression</label>
                <select class="app-expression-select"></select>
            </div>
            <% } %>
            <% if (name == 'soma') { %>
            <div class="form-group">
                <label title="Soma">Soma State</label>
                <select class="app-soma-select"></select>
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
            <% } else if (name == 'pause') { %>
            <div class="form-group">
                <label>Wait for topic</label>

                <input type="text" class="app-node-topic form-control" title="Topic name"
                       value="<%= typeof topic == 'undefined' ? '' : topic %>"/>
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
