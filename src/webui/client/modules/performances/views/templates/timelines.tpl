<div class="app-timelines-wrapper">
    <div class="app-edit-container clearfix">
        <div class="pull-left">
            <div class="btn-group">
                <button class="app-previous-button btn btn-default">Previous</button>
                <button class="app-next-button btn btn-default">Next</button>
            </div>

            <button class="app-add-new-button btn btn-info"><i
                    class="glyphicon glyphicon-plus-sign"></i> Add New
            </button>

            <button class="app-save-button btn btn-primary"><i
                    class="glyphicon glyphicon-floppy-save"></i> Save
            </button>
            <button class="app-done-button btn btn-warning"><i class="glyphicon glyphicon-ok"></i> Done</button>
        </div>

        <div class="performance-action-buttons">
            <button class="app-delete-button btn btn-danger" title="Delete">
                <i class="glyphicon glyphicon-trash"></i> Delete
            </button>
        </div>
    </div>

    <div class="clearfix">
        <div class="btn-group pull-left" role="group" aria-label="...">
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
            <button class="app-edit-button btn btn-primary"><i class="glyphicon glyphicon-edit"></i> Edit</button>
        </div>

        <div class="btn-group pull-right" role="group" aria-label="Zoom controls">
            <button class="app-zoom-out-button btn btn-default"><i class="glyphicon glyphicon glyphicon-zoom-out"></i></button>
            <button class="app-zoom-in-button btn btn-default"><i class="glyphicon glyphicon glyphicon-zoom-in"></i></button>
        </div>
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

    <ul class="app-node-context-menu dropdown-menu" role="menu">
        <li><a tabindex="-1" href="javascript:void(0)" data-action="copy">Copy</a></li>
        <li><a tabindex="-1" href="javascript:void(0)" data-action="invert">Invert selection</a></li>
        <li><a tabindex="-1" href="javascript:void(0)" data-action="delete">Delete</a></li>
    </ul>

    <ul class="app-timeline-context-menu dropdown-menu" role="menu">
        <li><a tabindex="-1" href="javascript:void(0)" data-action="paste">Paste</a></li>
        <li><a tabindex="-1" href="javascript:void(0)" data-action="select_all">Select All</a></li>
        <li><a tabindex="-1" href="javascript:void(0)" data-action="select_all_left">Select All to Left</a></li>
        <li><a tabindex="-1" href="javascript:void(0)" data-action="select_all_right">Select All to Right</a></li>
    </ul>
</div>

<div class="app-spinner-container"></div>