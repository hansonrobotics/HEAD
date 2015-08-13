define(['backgrid'], function (Backgrid) {
    return [{
        // enable the select-all extension
        name: "",
        cell: "select-row",
        headerCell: "select-all"
    }, {
        name: "name",
        label: "Name",
        // The cell type can be a reference of a Backgrid.Cell subclass, any Backgrid.Cell subclass instances like *id* above, or a string
        cell: "string" // This is converted to "StringCell" and a corresponding class in the Backgrid package namespace is looked up
    }, {
        name: "group",
        label: "Group",
        cell: "string"
    }, {
        name: "sort_no",
        label: "Sort NO",
        cell: "string"
    }, {
        name: "hardware",
        label: "Hardware",
        cell: "string"
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
        cell: "integer"
    }, {
        name: "init",
        label: "Init",
        cell: "integer"
    }, {
        name: "max",
        label: "Max",
        cell: "integer"
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
        cell: "number"
    }, {
        name: "lin_max",
        label: "Linear Max",
        cell: "number"
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
        cell: "string"
    }];
});