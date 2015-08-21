define(['backgrid', 'jquery', 'backgrid_text_cell'], function (Backgrid, $) {
    return [{
        // enable the select-all extension
        name: "",
        cell: "select-row",
        headerCell: "select-all"
    }, {
        name: "name",
        label: "Name",
        // The cell type can be a reference of a Backgrid.Cell subclass, any Backgrid.Cell subclass instances like *id* above, or a string
        cell: "string", // This is converted to "StringCell" and a corresponding class in the Backgrid package namespace is looked up
        formatter: _.extend({}, Backgrid.CellFormatter.prototype, {
            toRaw: function (val, model) {
                return val.length > 0 ? val : undefined;
            }
        })
    }, {
        name: "group",
        label: "Group",
        cell: "string"
    }, {
        name: "sort_no",
        label: "Sort NO",
        cell: "integer"
    }, {
        name: "hardware",
        label: "Hardware",
        cell: Backgrid.SelectCell.extend({
            optionValues: [['Pololu', 'pololu'], ['Dynamixel', 'dynamixel']]
        })
    }, {
        name: "motor_id",
        label: "Motor ID",
        cell: "integer"
    }, {
        name: "topic",
        label: "Topic",
        cell: "string"
    }, {
        name: "min",
        label: "Min",
        cell: "integer",
        formatter: _.extend({}, Backgrid.CellFormatter.prototype, {
            toRaw: function (val, model) {
                return $.isNumeric(val) && $.isNumeric(model.get('max')) && val > model.get('max') ? undefined : val;
            }
        })
    }, {
        name: "init",
        label: "Init",
        cell: "integer"
    }, {
        name: "max",
        label: "Max",
        cell: "integer",
        formatter: _.extend({}, Backgrid.CellFormatter.prototype, {
            toRaw: function (val, model) {
                return $.isNumeric(model.get('min')) && $.isNumeric(val) && val < model.get('min') ? undefined : val;
            }
        })
    }, {
        name: "speed",
        label: "Speed",
        cell: "number"
    }, {
        name: "acceleration",
        label: "Acceleration",
        cell: "number"
    }, {
        name: "parser",
        label: "Parser",
        cell: "string"
    }, {
        name: "parser_param",
        label: "Param",
        cell: "string"
    }, {
        name: "function",
        label: "Function",
        cell: Backgrid.SelectCell.extend({
            optionValues: [['Weighted Sum', 'weightedsum'], ['Linear', 'linear'], ['Other', '']]
        })
    }, {
        name: "lin_min",
        label: "Linear Min",
        cell: "number",
        formatter: _.extend({}, Backgrid.CellFormatter.prototype, {
            toRaw: function (val, model) {
                return $.isNumeric(val) && $.isNumeric(model.get('lin_max')) && val > model.get('lin_max') ? undefined : val;
            }
        })
    }, {
        name: "lin_max",
        label: "Linear Max",
        cell: "number",
        formatter: _.extend({}, Backgrid.CellFormatter.prototype, {
            toRaw: function (val, model) {
                return $.isNumeric(model.get('lin_min')) && $.isNumeric(val) && val < model.get('lin_min') ? undefined : val;
            }
        })
    }, {
        name: "max1",
        label: "Max1",
        cell: "number"
    }, {
        name: "imax1",
        label: "imax1",
        cell: "number"
    }, {
        name: "max2",
        label: "Max2",
        cell: "number"
    }, {
        name: "imax2",
        label: "Imax2",
        cell: "number"
    }, {
        name: "other_func",
        label: "Other Func (in JSON)",
        cell: "text"
    }];
});