<div>
    <ul class="app-tabs nav nav-tabs" role="tablist">
        <li role="presentation"><a class="app-settings-tab" data-target=".app-settings-tab-content"
                                   aria-controls="settings" role="tab" data-toggle="tab">Settings</a>
        </li>
        <li role="presentation"><a class="app-attention-tab" data-target=".app-attention-tab-content"
                                   aria-controls="attention" role="tab" data-toggle="tab">Attention regions</a></li>
    </ul>

    <div class="tab-content">
        <div role="tabpanel" class="app-settings-tab-content tab-pane fade active">
            <div class="form-group">
                <div class="input-group">
                    <div class="input-group-addon">Keywords</div>
                    <input type="text" class="app-keywords form-control" placeholder="Keywords">
                    <span class="input-group-btn"><button type="button"
                                                          class="app-save-keywords btn btn-primary">Save</button></span>
                </div>
            </div>
            <h4>Variables</h4>
            <div class="app-variable-template hidden">
                <div class="form-group">
                    <div class="input-group">
                        <div class="input-group-addon">Key:</div>
                        <input type="text" class="app-key-input form-control">
                        <div class="input-group-addon">Value:</div>
                        <input type="text" class="app-value-input form-control">
                        <span class="input-group-btn">
                            <button type="button" class="app-remove-variable-btn form-control btn btn-danger"><span
                                    class="glyphicon glyphicon-remove" aria-hidden="true"></span></button>
                        </span>
                    </div>
                </div>
            </div>
            <div class="app-variables-container"></div>

            <div class="btn-group">
                <button type="button" class="app-add-variable-btn btn btn-warning"><span
                        class="glyphicon glyphicon-plus" aria-hidden="true"></span> Add variable
                </button>
                <button class="app-save-variables-btn btn btn-primary"><span class="glyphicon glyphicon-save"
                                                                             aria-hidden="true"></span> Save
                </button>
            </div>
        </div>
        <div role="tabpanel" class="app-attention-tab-content tab-pane fade">
            <div class="app-select-areas-content"></div>
        </div>
    </div>
</div>
