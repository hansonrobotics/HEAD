<div class="panel panel-default">
    <div class="panel-heading">Logs</div>
    <div class="panel-body">
        <div class="input-group">
            <span class="input-group-addon" id="basic-addon1"><i class="glyphicon glyphicon-search"></i></span>
            <input type="text" class="form-control" placeholder="Search logs" aria-describedby="basic-addon1">
        </div>
        <br/>

        <div class="panel-group" id="accordion" role="tablist" aria-multiselectable="true">
            <div class="panel panel-default">
                <div class="panel-heading" role="tab" id="headingOne">
                    <h4 class="panel-title">
                        <a role="button" data-toggle="collapse" data-parent="#accordion" href="#collapseOne"
                           aria-expanded="true" aria-controls="collapseOne">
                            Blender
                            <span class="pull-right label label-info">5</span>
                        </a>
                    </h4>
                </div>
                <div id="collapseOne" class="panel-collapse collapse in" role="tabpanel" aria-labelledby="headingOne">
                    <div class="panel-body">
                        <div>﻿connect failed: No such file or directory</div>
                        <div class="text-danger">libGL error: pci id for fd 8: 80ee:beef, driver (null)</div>
                        <div class="text-danger">libGL error: core dri or dri2 extension not found</div>
                        <div class="text-danger">libGL error: failed to load driver: vboxvideo</div>
                        <div>read blend: /catkin_ws/src/blender_api/Eva.blend</div>
                        <div>Dependency cycle detected:</div>
                        <div>control depends on deform through Driver.</div>
                        <div>deform depends on control through Child Of.</div>
                    </div>
                </div>
            </div>
            <div class="panel panel-default">
                <div class="panel-heading" role="tab" id="headingTwo">
                    <h4 class="panel-title">
                        <a class="collapsed" role="button" data-toggle="collapse" data-parent="#accordion"
                           href="#collapseTwo" aria-expanded="false" aria-controls="collapseTwo">
                            Vision
                            <span class="pull-right label label-info">1</span>
                        </a>
                    </h4>
                </div>
                <div id="collapseTwo" class="panel-collapse collapse" role="tabpanel" aria-labelledby="headingTwo">
                    <div class="panel-body">
                        <div>logs</div>
                    </div>
                </div>
            </div>
        </div>
    </div>
</div>
