<div class="container">
    <div class="row">
        <div class="app-animations-column col-md-6">
            <div class="app-animations"></div>

            <hr/>
            <h3>Editing
                    <span class="pull-right">
                        <button class="app-add-animation btn btn-default"><span
                                class="glyphicon glyphicon-plus" area-hidden="true"></span> Add
                        </button>
                        <button class="app-copy-animation btn btn-warning">Copy</button>
                        <button class="app-delete-animation btn btn-danger">Delete</button>
                    </span>
            </h3>
            <hr/>

            <div class="app-animation-edit">
                <div class="alert alert-info">Please select an animation</div>
            </div>

            <h3>Frames</h3>
            <hr/>
            <div class="app-frames-container"></div>
            <div class="animation-action-buttons button-group">
                <button class="app-add-frame btn btn-default" title="Add Frame" type="button"><span
                        class="glyphicon glyphicon-plus" area-hidden="true"></span></button>
                <button class="app-save-frames btn btn-primary" type="button">Save</button>
            </div>
            <h4>Add expression as a frame</h4>
            <div class="app-expressions"></div>
        </div>

        <div class="app-motors-column app-admin col-md-6">
            <div class="app-controls app-editing">
                <button class="app-enable-torque btn btn-default">Enable Torque (dxl)</button>
                <button class="app-disable-torque btn btn-default">Disable Torque (dxl)</button>
                <button class="app-read-values btn btn-default">Read Values (dxl)</button>
                <hr/>
            </div>
            <div class="app-motors app-editing"></div>
        </div>
    </div>
</div>
