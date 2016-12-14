<h2>Timeline</h2>

<div class="app-edit-container clearfix">
    <div class="performance-name-container pull-left">
        <div class="input-group">
            <div class="input-group-addon">Name</div>
            <input class="app-performance-name form-control" type="text"/>
            <button class="app-save-button btn btn-primary"><i
                    class="glyphicon glyphicon-floppy-save"></i> Save
            </button>
        </div>
    </div>

    <div class="performance-action-buttons btn-group">
        <button class="app-delete-button btn btn-danger" title="Delete">
            <i class="glyphicon glyphicon-trash"></i> Delete
        </button>
        <button class="app-clear-button btn btn-warning" title="Clear">
            <i class="fa fa-eraser"></i>
            Clear
        </button>
        <button class="app-done-button btn btn-primary"><i class="glyphicon glyphicon-ok"></i> Done</button>

        <button class="app-yes-button btn btn-default" disabled="disabled">Save changes?</button>
        <button class="app-yes-button btn btn-success"><i class="glyphicon glyphicon-ok"></i> Yes</button>
        <button class="app-no-button btn btn-danger"><i class="glyphicon glyphicon-remove"></i> No</button>
        <button class="app-cancel-button btn btn-warning"><i class="glyphicon glyphicon-ban-circle"></i> Cancel</button>
    </div>
</div>

<div class="btn-group" role="group" aria-label="...">
    <button class="app-run-button btn btn-default" title="Run"><i
            class="glyphicon glyphicon-play-circle"></i> Run
    </button>
    <button class="app-pause-button btn btn-default" title="Pause"><i
            class="glyphicon glyphicon-pause"></i> Pause
    </button>
    <button class="app-resume-button btn btn-default" title="Resume"><i
            class="glyphicon glyphicon-play-circle"></i> Resume
    </button>
    <button class="app-auto-pause-button btn btn-default" title="Auto-pause"><i class="fa fa-pause-circle"
                                                                                aria-hidden="true"></i>
        Auto-pause
    </button>
    <button class="app-loop-button btn btn-default" title="Loop"><i
            class="glyphicon glyphicon-repeat"></i> Loop
    </button>
    <button class="app-stop-button btn btn-default" title="Stop"><i
            class="glyphicon glyphicon-stop"></i> Stop
    </button>
    <button class="app-frame-count btn btn-primary" title="Frame count" disabled="disabled">0</button>
</div>

<div class="app-timelines-container clearfix">
    <div class="app-scroll-container">
        <div class="app-timelines">
            <div class="app-run-indicator">
                <div class="app-current-time">
                    <div><i class="fa fa-arrows-h"></i></div>
                </div>
            </div>
        </div>
        <svg class="app-time-axis"></svg>
    </div>
</div>

<div class="app-node-settings-container"></div>
