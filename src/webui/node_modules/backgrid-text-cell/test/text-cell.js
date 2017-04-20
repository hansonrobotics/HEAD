/*
  backgrid
  http://github.com/wyuenho/backgrid

  Copyright (c) 2013 Jimmy Yuen Ho Wong and contributors
  Licensed under the MIT license.
*/
describe("A TextCell", function () {

  var cell;

  beforeEach(function () {
    cell = new Backgrid.Extension.TextCell({
      model: new Backbone.Model({
        name: "name"
      }),
      column: {
        name: "name",
        cell: "text"
      }
    });

  });

  it("applies a text-cell class to the cell", function () {
    cell.render();
    expect(cell.$el.hasClass("text-cell")).toBe(true);
  });

});

describe("A TextareaEditor", function () {

  var editor;
  var backgridEditedTriggerCount;
  var backgridEditedTriggerArgs;
  var backgridErrorTriggerCount;
  var backgridErrorTriggerArgs;

  beforeEach(function () {
    editor = new Backgrid.Extension.TextareaEditor({
      formatter: new Backgrid.Extension.TextCell.prototype.formatter,
      model: new Backbone.Model({
        name: "name <script></script>"
      }),
      column: {
        name: "name",
        cell: "text"
      }
    });

    backgridEditedTriggerCount = 0;
    editor.model.on("backgrid:edited", function () {
      backgridEditedTriggerCount++;
      backgridEditedTriggerArgs = [].slice.call(arguments);
    });

    backgridErrorTriggerCount = 0;
    editor.model.on("backgrid:error", function () {
      backgridErrorTriggerCount++;
      backgridErrorTriggerArgs = [].slice.call(arguments);
    });

    $("body").append(editor.el);

    editor.render();
  });

  afterEach(function () {
    editor.remove();
  });

  it("renders a dialog form with a textarea, a submit button and close button according to config", function () {
    expect(editor.$el.find("form").length).toBe(1);
    expect(editor.$el.find("form textarea").length).toBe(1);
    expect(editor.$el.find("form textarea").html()).toBe("name &lt;script&gt;&lt;/script&gt;");
    expect(editor.$el.find("form input[type=submit]").length).toBe(1);
    expect(editor.$el.find("button.close").length).toBe(1);
    expect(editor.$el.find("form textarea").prop("cols")).toBe(Backgrid.Extension.TextareaEditor.prototype.cols);
    expect(editor.$el.find("form textarea").prop("rows")).toBe(Backgrid.Extension.TextareaEditor.prototype.rows);
  });

  it("asks for confirmation when the textarea has text added during canceling", function (done) {
    var oldConfirm = window.confirm;
    var confirmArgs;

    window.confirm = function () {
      confirmArgs = [].slice.call(arguments);
      return false;
    };
    editor.render();
    editor.$el.find("textarea").val("name\r");
    editor.$el.find(".close").click();
    expect(confirmArgs[0]).toBe("Would you like to save your changes?");
    done();

    expect(editor.model.get(editor.column.get("name"))).toBe("name\n");
    expect(backgridEditedTriggerCount).toBe(1);
    expect(backgridEditedTriggerArgs[0]).toBe(editor.model);
    expect(backgridEditedTriggerArgs[1]).toBe(editor.column);
    expect(backgridEditedTriggerArgs[2].passThru()).toBe(true);

    window.confirm = oldConfirm;
  });

  it("asks for confirmation when the textarea was cleaned during canceling", function (done) {
    var oldConfirm = window.confirm;
    var confirmArgs;

    confirmArgs = null;
    window.confirm = function () {
      confirmArgs = [].slice.call(arguments);
      return false;
    };
    editor.model.unset("name");
    editor.render();
    editor.$el.find(".close").click();
    expect(confirmArgs).toBeNull();
    done();

    expect(backgridEditedTriggerCount).toBe(3);
    expect(editor.model.get(editor.column.get("name"))).toBeUndefined();
    expect(backgridEditedTriggerArgs[0]).toBe(editor.model);
    expect(backgridEditedTriggerArgs[1]).toBe(editor.column);
    expect(backgridEditedTriggerArgs[2].passThru()).toBe(true);

    window.confirm = oldConfirm;
  });

  it("saves the text from the textarea to the model and trigger 'backgrid:edited' from the model when the form is submitted", function (done) {
    editor.$el.find("textarea").val("another name");
    editor.$el.find("form").submit();
    done();

    expect(backgridEditedTriggerCount).toBe(1);
    expect(backgridEditedTriggerArgs[0]).toBe(editor.model);
    expect(backgridEditedTriggerArgs[1]).toBe(editor.column);
    expect(backgridEditedTriggerArgs[2].passThru()).toBe(true);
    expect(editor.model.get(editor.column.get("name"))).toBe("another name");
  });

  it("triggers 'backgrid:error' from the model if the formatter returns undefined", function () {
    editor.formatter.toRaw = function () {};
    editor.$el.find("textarea").val("another name");
    editor.$el.find("form").submit();
    expect(backgridErrorTriggerCount).toBe(1);
    expect(backgridErrorTriggerArgs[0]).toBe(editor.model);
    expect(backgridErrorTriggerArgs[1]).toBe(editor.column);
    expect(backgridErrorTriggerArgs[2]).toBe("another name");
  });
});
