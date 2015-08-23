<div class="container">
    <div class="row">
        <div class="col-md-9">
            <h4>Engagement loop mode</h4>

            <div class="btn-group" role="group" aria-label="Engagement loop mode">
                <button type="button" class="btn btn-primary">On</button>
                <button type="button" class="btn btn-default">Off</button>
                <button type="button" class="btn btn-default">Tradeshow</button>
                <button type="button" class="btn btn-default">Stage</button>
                <button type="button" class="btn btn-default">Companion</button>
            </div>
        </div>
        <div class="col-md-3">
            <h4>Transitions</h4>

            <div class="btn-group" role="group" aria-label="Sleep, wake up">
                <button type="button" class="btn btn-default">Sleep</button>
                <button type="button" class="btn btn-default">Wake</button>
            </div>
        </div>
    </div>
    <div class="row">
        <div class="col-md-12">
            <h4>Performances</h4>

            <form class="form-inline" style="margin-bottom: 10px">
                <div class="form-group">
                    <label class="sr-only" for="exampleInputAmount">Repeat every</label>

                    <div class="input-group">
                        <div class="input-group-addon">Repeat every</div>
                        <input type="text" value="3" class="form-control" id="exampleInputAmount" placeholder="">

                        <div class="input-group-addon">second(s)</div>
                    </div>
                </div>
            </form>

            <div class="btn-group" role="group" aria-label="Performances">
                <button type="button" class="btn btn-default">Introduce</button>
                <button type="button" class="btn btn-default">Tell a joke</button>
                <button type="button" class="btn btn-default">Do something</button>
                <button type="button" class="btn btn-default">Do something else</button>
            </div>
        </div>
    </div>
    <div class="row">
        <div class="col-md-12">
            <h4>Timeline Editor</h4>

            <div class="app-nodes">
                <div class="app-node label label-info">Emotion</div>
                <div class="app-node label label-danger">Gesture</div>
                <div class="app-node label label-success">Speech</div>
                <div class="app-node label label-primary">LookAt</div>
                <div class="app-node label label-danger">GazeAt</div>
                <div class="app-node label label-warning">FaceTracking</div>
                <div class="app-node label label-danger">Interaction</div>
            </div>

            <div class="app-timeline-editor">
                <div class="app-timeline"></div>
                <div class="app-timeline"></div>
                <div class="app-timeline"></div>
                <button class="app-add-timeline-button btn btn-default btn-block btn-sm" title="Add timeline"><i
                            class="glyphicon glyphicon-plus"></i> add timiline</button>
            </div>

            <div class="app-node-settings row">
                <div class="col-md-4">
                    <label for="exampleInputEmail2">Emotion</label>
                    <select name="" class="form-control">
                        <option value="1">Smile</option>
                    </select>
                </div>

                <div class="col-md-4">
                    <label for="exampleInputName2">Duration</label>

                    <div class="app-slider-demo"></div>
                </div>
                <div class="col-md-4">
                    <label for="exampleInputName2">Magnitude</label>

                    <div class="app-slider-demo"></div>
                </div>
            </div>

            <div class="btn-group" role="group" aria-label="...">
                <button class="app-add-timeline-button btn btn-default" title="Save"><i
                            class="glyphicon glyphicon-floppy-save"></i> Save
                </button>
            </div>

            <div class="btn-group" role="group" aria-label="...">
                <button class="app-add-timeline-button btn btn-default" title="Save"><i
                            class="glyphicon glyphicon-play-circle"></i> Run
                </button>
                <button class="app-add-timeline-button btn btn-default" title="Save"><i
                            class="glyphicon glyphicon-repeat"></i> Loop
                </button>
                <button class="app-add-timeline-button btn btn-default" title="Save"><i
                            class="glyphicon glyphicon-remove"></i> Clear
                </button>
            </div>

        </div>
    </div>
    <div class="row">
        <div class="col-md-12">
            <div class="clearfix">
                <h4 class="pull-left">Event queue</h4>
                <button id="app-clear-performance-queue" class="pull-right btn btn-default">
                    <span class="glyphicon glyphicon-trash"></span>
                    Clear all
                </button>
            </div>

            <ul id="app-performance-queue" class="list-group">
                <li class="list-group-item">
                    <span class="pull-left ui-icon ui-icon-arrowthick-2-n-s"></span>
                    Introduce
                    <div class="pull-right">
                        <span class="label label-primary">duration: 1 sec</span>

                        <button type="button" class="pull-right btn btn-danger">
                            <span class="glyphicon glyphicon-trash"></span>
                        </button>
                    </div>
                </li>
                <li class="list-group-item">
                    <span class="pull-left ui-icon ui-icon-arrowthick-2-n-s"></span>
                    Tell a joke
                    <div class="pull-right">
                        <span class="label label-primary">duration: 3 sec</span>

                        <button type="button" class="pull-right btn btn-danger">
                            <span class="glyphicon glyphicon-trash"></span>
                        </button>
                    </div>
                </li>
                <li class="list-group-item">
                    <span class="pull-left ui-icon ui-icon-arrowthick-2-n-s"></span>
                    Do something else
                    <div class="pull-right">
                        <span class="label label-primary">duration: 2 sec</span>

                        <button type="button" class="pull-right btn btn-danger">
                            <span class="glyphicon glyphicon-trash"></span>
                        </button>
                    </div>
                </li>
            </ul>
        </div>
    </div>
</div>
