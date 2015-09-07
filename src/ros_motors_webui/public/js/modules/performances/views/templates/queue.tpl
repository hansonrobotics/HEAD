<div class="row">
    <div class="col-md-12">
        <h4>Event queue</h4>
        <ul class="app-performance-queue list-group"></ul>

        <div class="clearfix">
            <button class="app-run btn btn-default">
                <span class="glyphicon glyphicon-play"></span>
                Run
            </button>
            <button class="app-pause btn btn-default">
                <span class="glyphicon glyphicon-pause"></span>
                Pause
            </button>
            <button class="app-clear pull-right btn btn-default">
                <span class="glyphicon glyphicon-trash"></span>
                Clear all
            </button>
        </div>

        <li class="app-performance-template app-performance list-group-item">
            <span class="app-drag-handle pull-left ui-icon ui-icon-arrowthick-2-n-s"></span>
            <span class="app-name"></span>
            <div class="pull-right">
                <span class="label label-primary">duration: <span class="app-duration"></span> sec</span>

                <button type="button" class="pull-right btn btn-success">
                    <span class="app-edit glyphicon glyphicon-edit"></span>
                </button>

                <button type="button" class="pull-right btn btn-danger">
                    <span class="app-remove glyphicon glyphicon-trash"></span>
                </button>
            </div>
        </li>
    </div>
</div>