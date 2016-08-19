<div class="app-options-content">
    <div class="app-node-properties col-sm-12">
        <div class="form-group">
            <label>Start time</label>

            <div class="input-group">
                <input type="text" class="app-node-start-time form-control" title="Start time"/>

                <div class="input-group-addon">s</div>
            </div>
        </div>

        <div class="form-group" data-node-property="duration">
            <label>Duration</label>

            <div class="input-group">
                <input type="text" class="app-node-duration form-control" title="Duration"/>

                <div class="input-group-addon">s</div>
            </div>
        </div>

        <div class="form-group" data-node-property="chatbot">
            <label>Chatbot</label>
            <select class="app-chatbot-select" style="width: 100%;"></select>
        </div>

        <div class="form-group" data-node-property="chat_timeout_mode">
            <label>Timeout Mode</label>
            <select class="app-chat-timeout-mode-select" style="width: 100%;">
                <option value="each">Each turn</option>
                <option value="whole">Whole chat</option>
            </select>
        </div>

        <div class="form-group" data-node-property="timeout">
            <label>Timeout</label>
            <div class="input-group">
                <input type="number" class="app-node-timeout form-control" title="Timeout"/>
                <div class="input-group-addon">s</div>
            </div>
        </div>

        <div class="form-group" data-node-property="message">
            <label>Message</label>

            <div class="input-group">
                <input type="text" class="app-node-message-input form-control" title="Message"/>
            </div>
        </div>

        <div class="form-group" data-node-property="speed">
            <label>Speed <span class="app-speed-label pull-right label label-default"></span></label>

            <div class="app-speed-slider"></div>
        </div>

        <div class="form-group" data-node-property="interval">
            <label>Interval <span class="app-interval-label pull-right label label-default"></span></label>

            <div class="app-interval-slider"></div>
        </div>

        <div class="form-group" data-node-property="fps">
            <label title="FPS">FPS <span
                        class="app-fps-label pull-right label label-default"></span></label>
            <div class="app-fps-slider"></div>
        </div>

        <div class="form-group" data-node-property="pitch">
            <label title="FPS">Pitch <span
                        class="app-pitch-label pull-right label label-default"></span></label>
            <div class="app-pitch-slider"></div>
        </div>

        <div class="form-group" data-node-property="volume">
            <label title="FPS">Volume <span
                        class="app-volume-label pull-right label label-default"></span></label>
            <div class="app-volume-slider"></div>
        </div>

        <div class="form-group" data-node-property="blender_mode">
            <label>Blender Disabled</label>
            <select class="app-blender-mode-select">
                <option value="no">No</option>
                <option value="face">Face</option>
                <option value="all">All</option>
            </select>
        </div>

        <div class="form-group" data-node-property="magnitude">
            <label title="Magnitude">Magnitude <span
                        class="app-magnitude-label pull-right label label-default"></span></label>

            <div class="app-magnitide-slider"></div>
        </div>

        <div class="form-group" data-node-property="crosshair">
            <div class="app-crosshair"></div>
        </div>

        <div class="form-group" data-node-property="dialog_turns">
            <label>Dialog turns</label>
            <input type="number" class="app-dialog-turns-input form-control">
        </div>

        <div class="form-group" data-node-property="no_match">
            <label>No match response</label>
            <input type="text" class="app-no-match-input form-control">
        </div>

        <div class="form-group" data-node-property="no_speech">
            <label>No speech response</label>
            <input type="text" class="app-no-speech-input form-control">
        </div>

        <div class="form-group">
            <button class="app-create-button pull-right btn btn-primary">Create</button>
        </div>
    </div>
</div>
