<div class="app-slider-container">
    <% if (typeof groupLabel != 'undefined') { %>
        <h3 class="motor-group-title"><%= groupLabel %></h3>
    <% } %>
    <hr class="app-motors-show-on-edit"/>

    <div class="app-motor-info app-motors-show-on-edit">
        <span class="app-motor-drag-handle pull-left ui-icon ui-icon-arrowthick-2-n-s"></span>
        <strong>Topic:</strong>
        <span class="app-motor-topic-name"><%= topic %></span>
        <strong>, Motor ID:</strong>
        <span class="app-motor-id"><%= (typeof motor_id == 'undefined') ? "" : motor_id %></span>
    </div>

    <div>
        <div class="app-motors-hide-on-edit pull-left">
            <span class="app-motor-drag-handle pull-left ui-icon ui-icon-arrowthick-2-n-s"></span>
            <b class="app-slider-label-left"></b>
        </div>

        <div class="pull-right">
            <b class="app-motors-hide-on-edit app-slider-label-right"></b>
            <% if (typeof motor_id != 'undefined') { %>
            <div class="pull-left app-motors-show-on-edit btn-group" role="group">
                <button type="button" class="app-angles-button motor-action btn btn-default">Angles</button>
                <button type="button" class="app-calibration-button motor-action btn btn-default">Calibration</button>
            </div>
            <% } %>
            <button type="button" class="pull-right app-select-motor-button motor-action btn"><span class="glyphicon"
                                                                                                    aria-hidden="true"></span>
            </button>
        </div>
    </div>

    <div class="app-motors-show-on-edit">
        <div class="pull-left">
            <input value="<%= labelleft %>" class="app-motor-label form-control" type="text"/>
        </div>
    </div>

    <div class="app-slider-value-container"><span class="app-slider-value"></span> (<span
            class="app-slider-min-value"></span> to <span class="app-slider-max-value"></span>)
    </div>

    <div class="clearfix"></div>

    <div class="app-slider"></div>

    <div class="app-motors-show-on-edit">
        <button class="app-motors-set-min btn btn-default">Set Min</button>
        <button class="app-motors-set-max btn btn-default">Set Max</button>
        <button class="app-motors-set-default btn btn-default">Set Default</button>
    </div>
</div>