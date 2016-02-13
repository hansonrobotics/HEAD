<div class="row">
    <div class="col-xs-12">
        <div class="app-chat-settings-container pull-right">
            <div class="app-language-select btn-group" role="group" aria-label="...">
                <button type="button" data-lang="en" class="btn btn-default btn-sm active">EN</button>
                <button type="button" data-lang="zh" class="btn btn-default btn-sm">CN</button>
            </div>

            <div class="app-recognition-select btn-group" role="group" aria-label="...">
                <button type="button" data-method="webspeech" class="btn btn-default btn-sm active">webspeech
                </button>
                <button type="button" data-method="iflytek" class="btn btn-default btn-sm">iflytek</button>
            </div>

            <div class="app-noise-container">
                <button class="app-adjust-noise-button btn btn-default btn-sm" type="button" title="Adjust noise">
                    <span class="glyphicon glyphicon-refresh" aria-hidden="true"></span>
                </button>

                <div class="app-noise-slider">
                    <span class="app-noise-value"><span class="caption"></span>noise: <span
                            class="value">500</span></span>
                </div>
            </div>
        </div>
        <div class="app-scrollbar">
            <div class="app-messages"></div>
        </div>
    </div>
</div>

<div class="row">
    <div class="col-xs-12">
        <div class="app-interaction-footer">
            <div class="app-faces-container"></div>
            <div class="message-input-container input-group">
                <input type="text" class="app-message-input form-control"
                       placeholder="Type your message here..."/>

                <span class="input-group-btn">
                    <button class="app-send-button btn btn-primary">Send</button>
                    <button class="app-record-button btn btn-info"><i class="fa fa-microphone"></i></button>
                </span>
            </div>
        </div>
    </div>
</div>
