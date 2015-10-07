<% if (typeof groupLabel != 'undefined') { %>
    <h3 class="motor-group-title"><%= groupLabel %></h3>
<% } %>

<div>
    <div class="pull-left">
        <b class="app-slider-label-left"></b>
    </div>

    <div class="app-slider-value-container"><span class="app-slider-value"></span> (<span
                class="app-slider-min-value"></span> to <span
                class="app-slider-max-value"></span>)
    </div>

    <div class="pull-right">
        <b class="app-slider-label-right"></b>
    </div>
</div>

<div class="motor-slider-container clearfix">
    <div class="app-slider"></div>
    <div class="app-motor-status-indicator" data-toggle="popover" data-content="ok"></div>
</div>
