<div class="app-timeline-editor-container row">
    <div class="col-md-12">
        <h4>Timeline</h4>

        <div class="clearfix">
            <div class="performance-name-container pull-left">
                <div class="input-group">
                    <div class="input-group-addon">Name</div>
                    <input class="app-performance-name form-control" placeholder="Enter name" type="text"
                           value="<%= name %>"/>
                    <button class="app-save-button btn btn-primary" title="Save"><i
                            class="glyphicon glyphicon-floppy-save"></i> Save
                    </button>
                </div>
            </div>

            <div class="performance-action-buttons btn-group">
                <button class="app-clear-button btn btn-warning" title="Clear">
                    <i class="fa fa-eraser"></i>
                    Clear
                </button>
                <button class="app-delete-button btn btn-danger" title="Delete">
                    <i class="glyphicon glyphicon-trash"></i> Delete
                </button>
            </div>
        </div>

        <div class="app-nodes clearfix">
            <div class="app-node label label-info" data-name="emotion">Pose</div>
            <div class="app-node label label-danger" data-name="gesture">Animation</div>
            <div class="app-node label label-success" data-name="speech">Speech</div>
            <div class="app-node label label-primary" data-name="look_at">LookAt</div>
            <div class="app-node label label-warning" data-name="gaze_at">GazeAt</div>
            <div class="app-node label label-default" data-name="interaction">Interaction</div>
            <div class="app-node label label-default" data-name="pause">Pause</div>
            <div class="app-node label label-info" data-name="expression">Expression</div>
            <div class="app-node label label-default" data-name="speech_input">Wait for speech</div>
        </div>

        <div class="app-node-settings"></div>

        <div class="app-timelines-container clearfix">
            <div class="app-scroll-container">
                <div class="app-timelines">
                    <div class="app-run-indicator"><div class="app-current-time"><div></div></div></div>
                </div>
                <svg class="app-time-axis"></svg>
            </div>
        </div>

        <div class="btn-group" role="group" aria-label="...">
            <button class="app-run-button btn btn-default" title="Save"><i
                    class="glyphicon glyphicon-play-circle"></i> Run
            </button>
            <button class="app-stop-button btn btn-default" title="Stop"><i
                    class="glyphicon glyphicon-stop"></i> Stop
            </button>
            <button class="app-pause-button btn btn-default" title="Stop"><i
                    class="glyphicon glyphicon-pause"></i> Pause
            </button>
            <button class="app-resume-button btn btn-default" title="Stop"><i
                    class="glyphicon glyphicon-play-circle"></i> Resume
            </button>
            <button class="app-loop-button btn btn-default" title="Loop"><i
                    class="glyphicon glyphicon-repeat"></i> Loop
            </button>
            <button class="app-frame-count btn btn-primary" title="Frame count" disabled="disabled">0</button>
        </div>
    </div>
</div>
