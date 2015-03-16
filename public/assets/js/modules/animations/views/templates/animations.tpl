<% _.each(animations, function(item){ %>
    <button class="btn btn-default <%= selected_name ? 'active' : '' %>" data-name="<%= item.name %>"><%= item.name %></button>
<% }); %>