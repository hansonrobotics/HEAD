<% if (typeof groupLabel != 'undefined') { %>
<h2 class="app-motor-group-title"><%= groupLabel %>
    <button type="button" class="app-select-group-button btn-warning pull-right btn"><span
            class="glyphicon glyphicon-asterisk"
            aria-hidden="true"></span></button>
</h2>
<% } %>

<div class="clearfix">
    <div>
        <b class="app-slider-label-left"></b>

        <span class="app-slider-value-container">
            <span class="app-slider-value"></span> (<span class="app-slider-min-value"></span> to
            <span class="app-slider-max-value"></span>)
        </span>

        <div class="pull-right">
            <button type="button" class="app-select-motor-button btn"><span class="glyphicon"
                                                                            aria-hidden="true"></span></button>
        </div>
    </div>
    <div class="pull-right">
        <b class="app-slider-label-right"></b>

        <div class="app-dynamixel-button-tooltip" data-placement="left"
             title="Toggle dynamixel params">
            <button class="btn btn-sm btn-primary" type="button"
                    data-toggle="collapse"
                    data-target=".app-dynamixel-collapse-<%= uniqueId %>"
                    aria-expanded="false"
                    aria-controls="app-dynamixel-collapse">
                <span class="glyphicon glyphicon-chevron-down"></span>
            </button>
        </div>
    </div>
</div>

<div class="app-dynamixel-collapse app-dynamixel-collapse-<%= uniqueId %> collapse">
    <div class="well">
        <dl class="app-dynamixel-params dl-horizontal"></dl>
    </div>
</div>

<div class="motor-slider-container clearfix">
    <div class="app-slider"></div>
    <div class="app-motor-indicators">
        <div class="app-motor-load app-ok"></div>
        <div class="app-motor-temperature app-ok"></div>
        <div class="app-motor-status-indicator" data-placement="left" data-trigger="hover"></div>
    </div>
</div>
