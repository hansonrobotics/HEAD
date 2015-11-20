<div class="app-edit-container">
    <div class="row">
        <div class="col-md-12">
            <div class="form-inline">
                <div class="input-group">
                    <span class="input-group-addon">Name</span>
                    <input class="app-performance-name form-control" type="text" value="<%= name %>"/>
                </div>
                <button class="app-delete-button btn btn-default" title="Delete"><span
                            class="glyphicon glyphicon-trash"></span></button>
            </div>
        </div>
    </div>
</div>

<div class="app-timeline-editor-container row">
    <div class="col-md-12">
        <h4>Timeline<span class="app-edit-container"> Editor</span></h4>

        <div class="app-edit-container">
            <div class="app-nodes">
                <div class="app-node label label-info" data-name="emotion">Pose</div>
                <div class="app-node label label-danger" data-name="gesture">Animation</div>
                <div class="app-node label label-success" data-name="speech">Speech</div>
                <div class="app-node label label-primary" data-name="look_at">LookAt</div>
                <div class="app-node label label-warning" data-name="gaze_at">GazeAt</div>
                <div class="app-node label label-default" data-name="interaction">Interaction</div>
                <div class="app-node label label-default" data-name="pause">Pause</div>
                <!--<div class="app-node label label-default" data-name="speech_input">Wait for speech</div>-->
            </div>

            <div class="app-node-settings"></div>
        </div>

        <div class="app-timelines-container">
            <div class="app-scroll-container">
                <div class="app-timelines">
                    <div class="app-run-indicator"></div>
                </div>
                <svg class="app-time-axis"></svg>
            </div>
        </div>

        <div class="app-edit-container">
            <div class="btn-group" role="group" aria-label="...">
                <button class="app-save-button btn btn-default" title="Save"><i
                            class="glyphicon glyphicon-floppy-save"></i> Save
                </button>
            </div>

            <div class="btn-group" role="group" aria-label="...">
                <button class="app-run-button btn btn-default" title="Save"><i
                            class="glyphicon glyphicon-play-circle"></i> Run
                </button>
                <button class="app-loop-button btn btn-default" title="Loop"><i
                            class="glyphicon glyphicon-repeat"></i> Loop
                </button>
                <button class="app-stop-button btn btn-default" title="Stop"><i
                            class="glyphicon glyphicon-stop"></i> Stop
                </button>
                <button class="app-clear-button btn btn-default" title="Clear"><i
                            class="glyphicon glyphicon-remove"></i> Clear
                </button>
            </div>
        </div>
    </div>
</div>
