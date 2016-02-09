<div class="speech-input-container row">
    <div class="col-md-12">
        <div class="input-group">
            <input type="text" class="app-speech-input form-control" placeholder="Enter speech"/>
            <span class="input-group-btn">
                <button class="app-say-button btn btn-primary">Say</button>
            </span>
        </div>
    </div>
</div>
<div class="row">
    <div class="col-md-6">
        <h4>Animation mode:</h4>
        <button type="button" disabled="disabled" class="app-mode-status btn btn-primary">0</button>
        <button type="button" class="app-animation-mode-button btn btn-default" data-mode="0">Off</button>
        <button type="button" class="app-animation-mode-button btn btn-default" data-mode="20">Auto Face Tracking</button>
        <button type="button" class="app-animation-mode-button btn btn-default" data-mode="127">Full</button>
        <h4>Poses</h4>
        <div class="app-pose-buttons"></div>
    </div>
    <div class="col-md-6">
        <h4>Animations</h4>
        <div class="app-animation-buttons"></div>
        <div class="app-cross-hairs row">
            <div class="col-md-4">
                <h4>Head Control</h4>
                <div class="app-head-control"></div>
            </div>
            <div class="col-md-4">
                <h4>Eye Control</h4>
                <div class="app-eye-control"></div>
            </div>
        </div>
    </div>
</div>
