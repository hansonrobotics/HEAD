<div class="app-slider-container">
    <hr class="app-motors-show-on-edit"/>

    <div class="app-motor-info app-motors-show-on-edit">
        <strong>Topic:</strong>
        <span class="app-motor-topic-name"><%= topic %></span>
        <strong>, Motor ID:</strong>
        <span class="app-motor-id"><%= name %></span>
    </div>

    <span class="app-motor-drag-handle pull-right ui-icon ui-icon-arrowthick-2-n-s"></span>

    <div class="app-motors-hide-on-edit">
        <div class="pull-left">
            <b class="app-slider-label-left"><%= labelleft %></b>
        </div>

        <div class="pull-right">
            <b class="app-slider-label-right"><%= labelright %></b>
        </div>
    </div>

    <div class="app-motors-show-on-edit">
        <div class="pull-left">
            <input value="<%= labelleft %>" class="app-motor-label form-control" type="text"/>
        </div>
    </div>

    <div class="app-slider-value-container"><span class="app-slider-value"></span>° (<span class="app-slider-min-value"><%= min %></span>° to <span class="app-slider-max-value"><%= max %></span>°)</div>

    <div class="clearfix"></div>

    <div class="app-slider"></div>

    <div class="app-motors-show-on-edit">
        <button class="app-motors-set-min btn btn-default">Set Min</button>
        <button class="app-motors-set-max btn btn-default">Set Max</button>
        <button class="app-motors-set-default btn btn-default">Set Default</button>
    </div>
</div>