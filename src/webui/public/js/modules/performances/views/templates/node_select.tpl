<div class="app-node-content">
    <div data-node-property="animation" class="app-gesture-list"></div>
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

    <div class="form-group" data-node-property="responses">
        <label>Responses:</label>
        <div class="hidden">
            <div class="app-listen-response-template">
                <div class="form-group">
                    <div class="input-group">
                        <div class="input-group-addon">Input:</div>
                        <input type="text" class="form-control">
                        <div class="input-group-addon">Output:</div>
                        <input type="text" class="form-control">
                        <span class="input-group-btn">
                            <button type="button" class="app-remove-listen-response form-control btn btn-danger"><span class="glyphicon glyphicon-remove"
                                                                               aria-hidden="true"></span></button>
                        </span>
                    </div>
                </div>
            </div>
        </div>
        <div class="app-listen-response-list"></div>
        <div class="form-group">
            <button type="button" class="app-listen-add-response btn btn-success"><span class="glyphicon glyphicon-plus"
                                                               aria-hidden="true"></span> Add</button>
        </div>
    </div>
</div>
