<div class="row">
    <div class="col-md-3">
        <div class="input-group">
            <span class="input-group-addon">Name</span>
            <input class="app-performance-name form-control" type="text" value="<%= name %>"/>
        </div>
    </div>
    <div class="col-md-3">
        <div class="btn-group" role="group">
            <button class="app-delete-button btn btn-default">Delete</button>
        </div>
    </div>
</div>

<div class="app-timeline-editor-container row">
    <div class="col-md-12">
        <h4>Timeline Editor</h4>

        <div class="app-nodes">
            <div class="app-node label label-info" data-name="emotion">Emotion</div>
            <div class="app-node label label-danger" data-name="gesture">Gesture</div>
            <div class="app-node label label-success" data-name="speech">Speech</div>
            <div class="app-node label label-primary" data-name="look_at">LookAt</div>
        </div>

        <div class="app-node-settings"></div>

        <div class="app-timelines-container">
            <div class="app-scroll-container">
                <div class="app-timelines">
                    <div class="app-run-indicator"></div>
                </div>
            </div>
        </div>

        <div class="btn-group" role="group" aria-label="...">
            <button class="app-save-button btn btn-default" title="Save"><i
                        class="glyphicon glyphicon-floppy-save"></i> Save
            </button>
        </div>

        <div class="btn-group" role="group" aria-label="...">
            <button class="app-run-button btn btn-default" title="Save"><i
                        class="glyphicon glyphicon-play-circle"></i> Run
            </button>
            <!--<button class="app-loop-button btn btn-default" title="Save"><i-->
            <!--class="glyphicon glyphicon-repeat"></i> Loop-->
            <!--</button>-->
            <button class="app-clear-button btn btn-default" title="Save"><i
                        class="glyphicon glyphicon-remove"></i> Clear
            </button>
        </div>
    </div>
</div>
