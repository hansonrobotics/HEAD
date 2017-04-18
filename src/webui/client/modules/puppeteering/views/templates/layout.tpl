<div class="container-fluid">
    <div class="app-puppeteering-container row">
        <% for (let w of config) { %>
        <div class="app-puppeteering-column col-md-<%= w.size || parseInt(12 / config.length) %>">
            <% for (let row of w.rows) { %>
            <div class="row">
                <% for (let col of row.cols) { %>
                <div class="col-md-<%= col.size || parseInt(12 / row.cols.length) %>"
                     data-column="<%= col.type %>"></div>
                <% } %>
            </div>
            <% } %>
        </div>
        <% } %>
    </div>
</div>
