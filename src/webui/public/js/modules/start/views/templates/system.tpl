<div class="row">
    <div class="col-xs-6">
        CPU
        <div class="progress">
            <div class="progress-bar app-cpu" role="progressbar" style="width:0%">
                0%
            </div>
        </div>
    </div>
    <div class="col-xs-6">
        Memory
        <div class="progress">
            <div class="progress-bar app-mem" role="progressbar" style="width:0%">
                0%
            </div>
        </div>
    </div>
</div>

<div class="row">
    <div class="col-md-12">
        <h2>Hardware</h2>
    </div>
</div>

<div class="status-list-row row list-inline list-group app-checks"></div>

<div class="row">
    <div class="col-md-6 hidden">
        <div class="col-sm-6">
            Robot Name:
        </div>
        <div class="col-sm-6">
            <select class="form-control" id="robot">
                <option value="sophia">Sophia</option>
            </select>
        </div>
    </div>
    <div class="col-md-6 app-software-control app-software-stopped">
        <button class="app-start-button btn btn-default">Start</button>
    </div>
    <div class="col-md-6 app-software-control app-software-started">
        Robot is currently starting. Please wait a moment.<button class="app-stop-button btn btn-default"> Force Stop</button><br/>
    </div>
    <div class="col-md-6 app-software-control app-software-starting">
        Redirecting you to the Robot Control Panel.
        <button class="app-stop-button btn btn-default">Force Stop</button>
    </div>

</div>
