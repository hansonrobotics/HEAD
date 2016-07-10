<div class="app-node-content">
    <div data-node-property="animation" class="app-gesture-list">
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test test test test test test test test test test test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
        <div class="app-node-thumbnail"><span>test</span></div>
    </div>

    <div data-node-property="emotion" class="app-emotion-list"></div>
    <div data-node-property="expression" class="app-expression-list"></div>
    <div class="app-soma-list"></div>
    <div class="app-kfanimation-list"></div>

    <div class="form-group" data-node-property="language">
        <label title="Language">Language</label>
        <select class="app-lang-select">
            <option value="en">English</option>
            <option value="zh">Mandarin</option>
        </select>
    </div>

    <div class="form-group" data-node-property="text">
        <label title="Text">Text</label>
        <textarea class="app-node-text form-control" title="Text"></textarea>
    </div>

    <div class="app-attention-region-list"></div>

    <div class="form-group" data-node-property="topic">
        <label>Wait for topic</label>
        <input type="text" class="app-node-topic form-control" title="Topic name"/>
    </div>

    <div class="form-group" data-node-property="btree_mode">
        <label title="Mode">Mode</label>
        <select class="app-btree-mode-select">
            <option value="255">Full</option>
            <option value="207">FT Off</option>
            <option value="48">FT On</option>
        </select>
    </div>

    <div class="form-group" data-node-property="speech_event">
        <label>Speech event</label>
        <select class="app-speech-event-select">
            <option value="">None</option>
            <option value="listening">Listening</option>
            <option value="talking">Talking</option>
        </select>
    </div>

    <div class="form-group" data-node-property="timeout">
        <label>Timeout</label>
        <div class="input-group">
            <input type="number" class="app-node-timeout form-control" title="Timeout"/>
            <div class="input-group-addon">s</div>
        </div>
    </div>

    <div class="form-group" data-node-property="angle">
        <label>Angle <span class="app-hr-angle-label pull-right label label-default"></span></label>
        <div class="app-hr-angle-slider"></div>
    </div>
</div>

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

        <div class="form-group" data-node-property="kfmode">
            <label title="Arms">Blender Disabled</label>
            <select class="app-kfmode-select">
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

        <div class="form-group">
            <button class="app-create-button pull-right btn btn-primary">Create</button>
        </div>
    </div>
</div>
