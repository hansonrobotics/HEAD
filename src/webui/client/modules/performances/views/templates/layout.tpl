<div class="app-performances-region"></div>
<div class="app-timeline-region"></div>

<div class="app-queue-container row">
    <div class="col-md-12">
        <h2>Event sequence</h2>

        <div class="app-queue-region"></div>

        <div class="clearfix">
            <button class="app-clear btn btn-default">
                <span class="glyphicon glyphicon-trash"></span>
                Clear all
            </button>
        </div>

        <li class="app-performance-template app-performance list-group-item">
            <div class="controls-container">
                <span class="app-drag-handle pull-left ui-icon ui-icon-arrowthick-2-n-s"></span>
                <button type="button" class="app-play pull-left btn btn-primary">
                    <span class="glyphicon glyphicon-play"></span>
                </button>
            </div>

            <span class="app-description-container">
                <span class="app-name"></span>
                <span class="app-desc label label-alt"></span>
            </span>

            <div class="button-container">
                <span class="label label-primary"><span class="app-duration"></span> sec</span>

                <button type="button" class="app-edit btn btn-success">
                    <span class="glyphicon glyphicon-edit"></span>
                </button>

                <button type="button" class="app-remove btn btn-danger">
                    <span class="glyphicon glyphicon-trash"></span>
                </button>
            </div>
        </li>
    </div>
</div>

<div class="app-save-changes-confirmation modal fade" tabindex="-1" role="dialog">
    <div class="modal-dialog" role="document">
        <div class="modal-content">
            <div class="modal-body">
                <button type="button" class="close" data-dismiss="modal" aria-label="Close"><span aria-hidden="true">&times;</span></button>
                <h4 class="modal-title">Would you like to save the changes?</h4>
            </div>
            <div class="modal-footer">
                <button type="button" class="btn btn-default" data-dismiss="modal">Cancel</button>
                <button type="button" class="app-discard-changes btn btn-danger">No</button>
                <button type="button" class="app-save-changes btn btn-primary">Yes</button>
            </div>
        </div>
    </div>
</div>
