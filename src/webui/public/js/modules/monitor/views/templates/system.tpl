<div class="row">
    <div class="col-xs-4">
        CPU
        <div class="progress">
            <div class="progress-bar app-cpu" role="progressbar" style="width:0%">
                0%
            </div>
        </div>
    </div>
    <div class="col-xs-4">
        Memory
        <div class="progress">
            <div class="progress-bar app-mem" role="progressbar" style="width:0%">
                0%
            </div>
        </div>
    </div>
    <div class="col-xs-4">
        Blender FPS
        <div class="progress">
            <div class="progress-bar app-fps progress-bar-" role="progressbar"
                 style="width:0%">
                0
            </div>
        </div>
    </div>
</div>
<div class="row hidden">
    <div class="col-md-6">
        <div class="col-sm-6">
            Robot Name:
        </div>
        <div class="col-sm-6">
            <select class="form-control" id="robot">
                <option value="sophia">Sophia</option>
            </select>
        </div>
    </div>
    <div class="col-md-6">
        <button class="app-start-button btn btn-default">Start</button>
        <button class="app-restart-button btn btn-default">Restart</button>
        <button class="app-save-button btn btn-default">Shutdown</button>
    </div>
</div>
<div class="status-list-row row">
    <div class="col-xs-6">
        <h2>Hardware</h2>
        <ul class="list-group">
            <li class="list-group-item">
                USB2Dynamixel
                <span class="pull-right label status-item" data-status="usb2dynamixel"></span>
            </li>
            <li class="list-group-item">
                Pololu Board
                <span class="pull-right label status-item" data-status="pololu"></span>
            </li>
            <li class="list-group-item">
                Camera
                <span class="pull-right label status-item" data-status="camera"></span>
            </li>
        </ul>
    </div>
    <div class="col-xs-6">
        <h2>Software</h2>
        <ul class="list-group">
            <li class="list-group-item">
                ROS
                <span class="pull-right label status-item" data-status="ros"></span>
            </li>
            <li class="list-group-item">
                Blender
                <span class="pull-right label status-item" data-status="blender"></span>
            </li>
            <li class="list-group-item">
                Internet
                <span class="pull-right label status-item" data-status="internet"></span>
            </li>
        </ul>
    </div>
</div>

<div class="row">
    <div class="col-md-12">
        <h2>ROS Nodes</h2>
    </div>
</div>

<div class="status-list-row row list-inline list-group app-ros-nodes"></div>
