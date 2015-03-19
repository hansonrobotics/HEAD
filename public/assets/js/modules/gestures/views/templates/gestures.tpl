<div class="container">
    <div class="row">
        <div class="col-md-2">
            <h4>Demo: </h4>
            <button id="app-gesture-demo-start" type="button" class="btn btn-default">Start</button>
            <button id="app-gesture-demo-stop" type="button" class="btn btn-default">Stop</button>
        </div>
        <div class="col-md-10">
            <h4>Gestures: </h4>

            <div id="app-gesture-buttons"></div>
        </div>
    </div>
    <div class="row">
        <div class="col-md-12">
            <h4>Emotions: </h4>

            <ul id="app-emotion-sliders">
                <li id="app-emotion-slider-template" class="app-emotion-slider-container hidden">
                    <div class="pull-left">
                        <b class="app-slider-label-left"></b>
                    </div>

                    <div class="pull-right">
                        <b class="app-slider-label-right"></b>
                    </div>

                    <div class="app-slider-value-container"><span class="app-slider-value">0</span>° (<span
                            class="app-slider-min-value">-90</span>° to <span
                            class="app-slider-max-value">90</span>°)
                    </div>

                    <div class="app-slider"></div>
                </li>
            </ul>
        </div>
    </div>
</div>