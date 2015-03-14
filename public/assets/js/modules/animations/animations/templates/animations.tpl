<% _.each(animations, function(item){ %>
    <button class="btn btn-default" data-name="<%= item.name %>"><%= item.name %></button>
<% }); %>