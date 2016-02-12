<div class="row">
    <div class="col-md-12">
        <h4>Poses: </h4>

        <div class="row">
            <div class="col-md-6">
                <div class="hidden-xs">
                    <div class="pull-left">
                        <strong>Duration</strong>
                    </div>

                    <div class="app-slider-value-container">
                        <span class="app-duration-value"><%= duration.default %></span> seconds
                    </div>

                    <div class="app-duration-slider"></div>
                    <br/>
                </div>
            </div>
            <div class="col-md-6">
                <div class="hidden-xs">
                    <div class="pull-left">
                        <strong>Magnitude</strong>
                    </div>

                    <div class="app-slider-value-container">
                        <span class="app-magnitude-value"><%= magnitude.default * 100 %></span>%
                    </div>

                    <div class="app-magnitude-slider"></div>
                    <br/>
                </div>
            </div>
        </div>

        <div class="app-emotions-container"></div>
    </div>
</div>
