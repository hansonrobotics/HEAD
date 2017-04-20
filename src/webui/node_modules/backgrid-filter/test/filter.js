/*
  backgrid
  http://github.com/wyuenho/backgrid

  Copyright (c) 2013 Jimmy Yuen Ho Wong and contributors
  Licensed under the MIT @license.
*/
describe("A ServerSideFilter", function () {

  var ajax, collection;

  beforeEach(function () {
    collection = new (Backbone.Collection.extend({
      url: "http://www.example.com"
    }))([{id: 1}, {id: 2}]);

    ajax = $.ajax;
  });

  afterEach(function () {
    $.ajax = ajax;
  });

  it("render the compiled template supplied from the constructor", function () {
    var filter = new Backgrid.Extension.ServerSideFilter({
      template: _.template("<div>hello world!</div>")
    });
    filter.render();
    expect(filter.el.innerHTML.toLowerCase()).toBe("<div>hello world!</div>");
  });

  it("can render a search box with an optional name, placeholder or value", function () {
    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection,
      name: "name",
      value: "value",
      placeholder: "placeholder"
    });
    filter.render();
    expect(filter.searchBox().attr("name")).toBe("name");
    expect(filter.searchBox().attr("value")).toBe("value");
    expect(filter.searchBox().attr("placeholder")).toBe("placeholder");

    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection,
      name: "name"
    });
    filter.render();
    expect(filter.searchBox().attr("name")).toBe("name");
    expect(filter.searchBox().attr("value")).toBeUndefined();
    expect(filter.searchBox().attr("placeholder")).toBeUndefined();

    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection,
      value: "value"
    });
    filter.render();
    expect(filter.searchBox().attr("name")).toBe("q");
    expect(filter.searchBox().attr("value")).toBe("value");
    expect(filter.searchBox().attr("placeholder")).toBeUndefined();

    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection,
      placeholder: "placeholder"
    });
    filter.render();
    expect(filter.searchBox().attr("name")).toBe("q");
    expect(filter.searchBox().attr("value")).toBeUndefined();
    expect(filter.searchBox().attr("placeholder")).toBe("placeholder");

    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    expect(filter.searchBox().attr("name")).toBe("q");
    expect(filter.searchBox().attr("value")).toBeUndefined();
    expect(filter.searchBox().attr("placeholder")).toBeUndefined();
  });

  it("can fetch with a query on submit", function () {
    var url, data;
    $.ajax = function (settings) {
      url = settings.url;
      data = settings.data;
      settings.success([{id: 1}]);
    };
    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    filter.searchBox().val("query");
    filter.$el.submit();
    expect(url).toBe("http://www.example.com");
    expect(data).toEqual({q: "query"});
    expect(collection.length).toBe(1);
    expect(collection.at(0).toJSON()).toEqual({id: 1});

    filter = new Backgrid.Extension.ServerSideFilter({
      collection: new (Backbone.PageableCollection.extend({
        url: "http://www.example.com"
      }))(null)
    });
    filter.render();
    filter.searchBox().val("query");
    filter.$el.submit();
    expect(url).toBe("http://www.example.com");
    expect(data).toEqual({q: "query", page: 1, "per_page": 25});
    expect(collection.length).toBe(1);
    expect(collection.at(0).toJSON()).toEqual({id: 1});
  });

  it("fetches without the query parameter if the search box is empty", function () {
    var url, data;
    $.ajax = function (settings) {
      url = settings.url;
      data = settings.data;
      settings.success([{id: 1}]);
    };
    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    filter.searchBox().val("");
    filter.$el.submit();
    expect(url).toBe("http://www.example.com");
    expect(data).toEqual({});
    expect(collection.length).toBe(1);
    expect(collection.at(0).toJSON()).toEqual({id: 1});

    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: new (Backbone.PageableCollection.extend({
        url: "http://www.example.com"
      }))(null)
    });
    filter.render();
    filter.searchBox().val("");
    filter.$el.submit();
    expect(url).toBe("http://www.example.com");
    expect(data).toEqual({page: 1, "per_page": 25});
    expect(collection.length).toBe(1);
    expect(collection.at(0).toJSON()).toEqual({id: 1});
  });

  it("can persist the filter parameter on pagination", function () {
    var url, data;
    $.ajax = function (settings) {
      url = settings.url;
      data = settings.data;
      settings.success([{id: 2}]);
    };
    collection = new (Backbone.PageableCollection.extend({
      url: "http://www.example.com"
    }))([{id: 1}], {
      state: {
        pageSize: 1,
        totalRecords: 3
      },
      queryParams: {
        totalRecords: null,
        totalPages: null
      }
    });
    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    filter.searchBox().val("query");
    collection.getPage(2);
    expect(filter.searchBox().val()).toBe("query");
    expect(url).toBe("http://www.example.com");
    expect(data).toEqual({q: "query", page: 2, "per_page": 1});
    expect(collection.length).toBe(1);
    expect(collection.at(0).toJSON()).toEqual({id: 2});
  });

  it("goes back to the first page when the query changes", function () {
    var url, data;
    $.ajax = function (settings) {
      url = settings.url;
      data = settings.data;
      settings.success([{id: 3}]);
    };
    collection = new (Backbone.PageableCollection.extend({
      url: "http://www.example.com"
    }))([{id: 1}, {id: 2}], {
      state: {
        pageSize: 1,
        totalRecords: 3
      },
      queryParams: {
        totalRecords: null,
        totalPages: null
      }
    });
    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    collection.getPage(2);
    filter.searchBox().val("query");
    filter.$el.submit();
    expect(url).toBe("http://www.example.com");
    expect(data).toEqual({q: "query", page: 1, "per_page": 1});
    expect(collection.length).toBe(1);
    expect(collection.state.currentPage).toBe(1);
    expect(collection.state.totalRecords).toBe(3);
    expect(collection.at(0).toJSON()).toEqual({id: 3});

    collection = new (Backbone.PageableCollection.extend({
      url: "http://www.example.com"
    }))([{id: 1}, {id: 2}], {
      state: {
        firstPage: 0,
        pageSize: 1,
        totalRecords: 3
      },
      queryParams: {
        totalRecords: null,
        totalPages: null
      }
    });
    filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    collection.getNextPage();
    filter.searchBox().val("query").keyup();
    filter.$el.submit();
    expect(url).toBe("http://www.example.com");
    expect(data).toEqual({q: "query", page: 0, "per_page": 1});
    expect(collection.length).toBe(1);
    expect(collection.state.currentPage).toBe(0);
    expect(collection.state.totalRecords).toBe(3);
    expect(collection.at(0).toJSON()).toEqual({id: 3});
  });

  it("goes back to the first page when the clear button was clicked", function () {
    var url, data;
    $.ajax = function (settings) {
      url = settings.url;
      data = settings.data;
      settings.success([{id: 3}]);
    };

    collection = new (Backbone.PageableCollection.extend({
      url: "http://www.example.com"
    }))([{id: 1}, {id: 2}], {
      state: {
        pageSize: 1,
        totalRecords: 3
      },
      queryParams: {
        totalRecords: null,
        totalPages: null
      }
    });
    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    collection.getPage(2);
    filter.searchBox().val("query").keyup();
    filter.clearButton().click();
    filter.$el.submit();
    expect(url).toBe("http://www.example.com");
    expect(data).toEqual({page: 1, "per_page": 1});
    expect(collection.length).toBe(1);
    expect(collection.state.currentPage).toBe(1);
    expect(collection.state.totalRecords).toBe(3);
    expect(collection.at(0).toJSON()).toEqual({id: 3});

    collection = new (Backbone.PageableCollection.extend({
      url: "http://www.example.com"
    }))([{id: 1}, {id: 2}], {
      state: {
        firstPage: 0,
        pageSize: 1,
        totalRecords: 3
      },
      queryParams: {
        totalRecords: null,
        totalPages: null
      }
    });
    filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    collection.getPage(2);
    filter.searchBox().val("query").keyup();
    filter.clearButton().click();
    filter.$el.submit();
    expect(url).toBe("http://www.example.com");
    expect(data).toEqual({page: 0, "per_page": 1});
    expect(collection.length).toBe(1);
    expect(collection.state.currentPage).toBe(0);
    expect(collection.state.totalRecords).toBe(3);
    expect(collection.at(0).toJSON()).toEqual({id: 3});
  });

  it("can clear the search box and refetch upon clicking the cross", function () {
    spyOn(collection, "fetch");
    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    filter.searchBox().val("query");
    filter.clearButton().click();
    expect(filter.searchBox().val()).toBe("");
    expect(filter.clearButton().css("display")).toBe("none");
    collection.fetch.calls.reset();
  });

  it("shows the clear button when the search box has text entered, hides it otherwise", function () {
    var filter = new Backgrid.Extension.ServerSideFilter({
      collection: collection
    });
    filter.render();
    expect(filter.clearButton().css("display")).toBe("none");

    filter.searchBox().val("query").keyup();
    expect(filter.clearButton().css("display")).not.toBe("none");

    filter.searchBox().val(null).keyup();
    expect(filter.clearButton().css("display")).toBe("none");
  });

});

describe("A ClientSideFilter", function () {

  var collection;

  beforeEach(function () {
    collection = new (Backbone.Collection.extend({
      url: "http://www.example.com"
    }))([{id: 1, name: "alice"},
         {id: 2, name: "alicia"},
         {id: 3, name: "bob"}]);
  });

  it("can perform a regex search on keydown and submit, and cancel on clicking the clear button", function (done) {
    var filter = new Backgrid.Extension.ClientSideFilter({
      collection: collection,
      fields: ["name"]
    });
    filter.render();

    expect(collection.length).toBe(3);
    expect(collection.at(0).id).toBe(1);
    expect(collection.at(1).id).toBe(2);
    expect(collection.at(2).id).toBe(3);

    filter.searchBox().val("bob").keydown().keyup();

    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(3);

    filter.clearButton().click();

    done();

    expect(collection.length).toBe(3);
    expect(collection.at(0).id).toBe(1);
    expect(collection.at(1).id).toBe(2);
    expect(collection.at(2).id).toBe(3);


    filter.searchBox().val("ALICE");
    filter.$el.submit();

    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(1);

    filter.searchBox().val("al").keydown().keyup();

    done();

    expect(collection.length).toBe(2);
    expect(collection.at(0).id).toBe(1);
    expect(collection.at(1).id).toBe(2);

    filter.searchBox().val("alic bob").keydown().keyup();

    done();

    expect(collection.length).toBe(3);
    expect(collection.at(0).id).toBe(1);
    expect(collection.at(1).id).toBe(2);
    expect(collection.at(2).id).toBe(3);
  });

  it("will reflect in the search result when a new model is added to the collection", function (done) {
    var filter = new Backgrid.Extension.ClientSideFilter({
      collection: collection,
      fields: ["name"]
    });
    filter.render();
    collection.add({id: 4, name: "doug"});
    filter.searchBox().val("doug").keydown().keyup();

    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(4);
  });

  it("will reflect in the search result when a model is removed from the collection", function (done) {
    var filter = new Backgrid.Extension.ClientSideFilter({
      collection: collection,
      fields: ["name"]
    });
    filter.render();
    collection.remove(collection.at(0));
    filter.searchBox().val("alice").keydown().keyup();

    done()

    expect(collection.length).toBe(0);
    expect(filter.shadowCollection.length).toBe(2);
    expect(filter.shadowCollection.at(0).id).toBe(2);
    expect(filter.shadowCollection.at(1).id).toBe(3);
  });

  it("will reflect in the search result when a model attribute is changed", function (done) {
    var filter = new Backgrid.Extension.ClientSideFilter({
      collection: collection,
      fields: ["name"]
    });
    filter.render();
    filter.collection.at(0).set("name", "charlie");
    filter.searchBox().val("charlie").keydown().keyup();

    done();

    expect(collection.length).toBe(1);
    expect(filter.shadowCollection.at(0).id).toBe(1);
    expect(filter.shadowCollection.at(0).get("name")).toBe("charlie");
    expect(filter.collection.at(0).get("name")).toBe("charlie");
  });

  it("will reflect in the search result when the collection is reset", function (done) {
    var filter = new Backgrid.Extension.ClientSideFilter({
      collection: collection,
      fields: ["name"]
    });
    filter.render();
    filter.collection.reset([{id: 4, name: "charlie"}, {id: 5, name: "doug"}]);
    filter.searchBox().val("").keydown().keyup();

    done();

    expect(collection.length).toBe(2);
    expect(filter.shadowCollection.at(0).id).toBe(4);
    expect(filter.shadowCollection.at(1).id).toBe(5);
    expect(filter.shadowCollection.at(0).get("name")).toBe("charlie");
    expect(filter.shadowCollection.at(1).get("name")).toBe("doug");
    expect(filter.collection.at(0).id).toBe(4);
    expect(filter.collection.at(1).id).toBe(5);
    expect(filter.collection.at(0).get("name")).toBe("charlie");
    expect(filter.collection.at(1).get("name")).toBe("doug");
  });

  it("goes back to the first page when the query changes", function (done) {
    collection = new Backbone.PageableCollection([
      {id: 1, name: "alice"},
      {id: 2, name: "bob"}], {
        state: {
          pageSize: 1,
        },
        mode: "client"
      });

    var filter = new Backgrid.Extension.ClientSideFilter({
      collection: collection,
      fields: ["name"]
    });
    filter.render();
    collection.getPage(2);
    filter.searchBox().val("bob").keydown().keyup();

    done();

    expect(collection.state.currentPage).toBe(1);
    expect(collection.length).toBe(1);
    expect(filter.shadowCollection.at(0).id).toBe(1);
    expect(filter.shadowCollection.at(1).id).toBe(2);
    expect(collection.state.currentPage).toBe(1);
    expect(collection.state.totalRecords).toBe(1);
    expect(collection.at(0).toJSON()).toEqual({id: 2, name: "bob"});
  });

  it("goes back to the first page when the clear button was clicked", function (done) {
    collection = new Backbone.PageableCollection([
      {id: 1, name: "alice"},
      {id: 2, name: "amanda"}], {
        state: {
          pageSize: 1,
        },
        mode: "client"
      });

    var filter = new Backgrid.Extension.ClientSideFilter({
      collection: collection,
      fields: ["name"]
    });
    filter.render();
    filter.searchBox().val("a").keydown().keyup();
    collection.getPage(2);
    filter.clearButton().click();

    done();

    expect(collection.state.currentPage).toBe(1);
    expect(collection.length).toBe(1);
    expect(filter.shadowCollection.at(0).id).toBe(1);
    expect(filter.shadowCollection.at(1).id).toBe(2);
    expect(collection.state.totalRecords).toBe(2);
    expect(collection.at(0).toJSON()).toEqual({id: 1, name: "alice"});
  });

  it("can clear the search box and reset the collection when the clear button is clicked", function (done) {
    var filter = new Backgrid.Extension.ClientSideFilter({
      collection: collection,
      fields: ["name"]
    });
    filter.render();

    filter.searchBox().val("alice").keydown().keyup();

    done();

    expect(collection.length).toBe(1);

    filter.clearButton().click();

    done();

    expect(collection.length).toBe(3);
    expect(filter.searchBox().val()).toBe('');
    expect(collection.at(0).id).toBe(1);
    expect(collection.at(1).id).toBe(2);
    expect(collection.at(2).id).toBe(3);
  });

  it("shows the clear button when the search box has text entered, hides it otherwise", function (done) {
    var filter = new Backgrid.Extension.ClientSideFilter({
      collection: collection,
      fields: ["name"]
    });
    filter.render();
    filter.searchBox().val("bob").keydown().keyup();

    done();

    expect(collection.length).toBe(1);

    expect(filter.clearButton().css("display")).not.toBe("none");
    filter.searchBox().val(null).keydown().keyup();

    done();

    expect(collection.length).toBe(3);
    expect(filter.clearButton().css("display")).toBe("none");
  });

});

describe("A LunrFilter", function () {

  var collection;

  beforeEach(function () {
    collection = new (Backbone.Collection.extend({
      url: "http://www.example.com"
    }))([{id: 1, name: "alice", bio: "a fat cat sat on a mat and ate a fat rat"},
         {id: 2, name: "bob", bio: "he is fat but does not crap"}]);
  });

  it("can perform a full-text search on keydown and submit, and cancel on clicking the clear button", function (done) {
    var filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });
    filter.render();
    expect(collection.length).toBe(2);
    expect(collection.at(0).id).toBe(1);
    expect(collection.at(1).id).toBe(2);


    filter.searchBox().val("crap").keydown().keyup();
    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(2);

    filter.clearButton().click();
    done();

    expect(collection.length).toBe(2);
    expect(collection.at(0).id).toBe(1);
    expect(collection.at(1).id).toBe(2);

    filter.searchBox().val("alice");
    filter.$el.submit();

    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(1);

    filter.searchBox().val("fat").keydown().keyup();

    done();

    expect(collection.length).toBe(2);
    expect(collection.at(0).id).toBe(2);
    expect(collection.at(1).id).toBe(1);
  });

  it("will reindex on reset", function (done) {
    var filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });

    filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });
    filter.render();
    collection.reset([{id: 3, name: "charlie", bio: "The cat scared the bat and sat on the mat."},
                      {id: 4, name: "doug", bio: "The snail hid in his shell. It heard a bell. He went to see it was the church bell."}]);

    filter.searchBox().val("crap").keydown().keyup();

    done();

    expect(collection.length).toBe(0);

    filter.searchBox().val("alice").keydown().keyup();

    done();

    expect(collection.length).toBe(0);


    filter.searchBox().val("charlie").keydown().keyup();

    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(3);

    filter.searchBox().val("doug").keydown().keyup();

    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(4);
  });

  it("will update the index on add", function (done) {
    var filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });
    filter.render();
    collection.add({id: 3, name: "charlie", bio: "The cat scared the bat and sat on the mat."});

    filter.searchBox().val("charlie").keydown().keyup();

    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(3);
  });

  it("will update the index on remove", function (done) {
    var filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });
    filter.render();
    collection.remove(collection.last());

    filter.searchBox().val("bob").keydown().keyup();

    done();

    expect(collection.length).toBe(0);

    filter.searchBox().val("alice").keydown().keyup();

    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(1);
  });

  it("will update the index on change", function (done) {
    var filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });
    filter.render();
    collection.at(0).set("name", "charlie");

    filter.searchBox().val("alice").keydown().keyup();

    done();

    expect(collection.length).toBe(0);

    filter.searchBox().val("charlie").keydown().keyup();

    done();

    expect(collection.length).toBe(1);
    expect(collection.at(0).id).toBe(1);
  });

  it("goes back to the first page when the query changes", function (done) {
    collection = new Backbone.PageableCollection([
      {id: 1, name: "alice", bio: "a fat cat sat on a mat and ate a fat rat"},
      {id: 2, name: "bob", bio: "he is fat but does not crap"}
    ], {
      state: {
        pageSize: 1,
      },
      mode: "client"
    });

    var filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });
    filter.render();
    collection.getPage(2);
    filter.searchBox().val("fat").keydown().keyup();

    done();

    expect(collection.state.currentPage).toBe(1);
    expect(collection.length).toBe(1);
    expect(filter.shadowCollection.at(0).id).toBe(1);
    expect(filter.shadowCollection.at(1).id).toBe(2);
    expect(collection.state.currentPage).toBe(1);
    expect(collection.state.totalRecords).toBe(2);
    expect(collection.at(0).id).toBe(2);
  });

  it("goes back to the first page when the clear button was clicked", function (done) {
    collection = new Backbone.PageableCollection([
      {id: 1, name: "alice", bio: "a fat cat sat on a mat and ate a fat rat"},
      {id: 2, name: "bob", bio: "he is fat but does not crap"}
    ], {
      state: {
        pageSize: 1,
      },
      mode: "client"
    });

    var filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });
    filter.render();
    filter.searchBox().val("fat").keydown().keyup();
    collection.getPage(2);
    filter.clearButton().click();

    done();

    expect(collection.state.currentPage).toBe(1);
    expect(collection.length).toBe(1);
    expect(filter.shadowCollection.at(0).id).toBe(1);
    expect(filter.shadowCollection.at(1).id).toBe(2);
    expect(collection.state.totalRecords).toBe(2);
    expect(collection.at(0).id).toBe(1);
  });

  it("can clear the search box and reindex upon clicking the clear button", function (done) {
    var filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });
    filter.render();

    filter.searchBox().val("crap").keydown().keyup();

    done();

    expect(collection.length).toBe(1);

    filter.clearButton().click();

    done();

    expect(collection.length).toBe(2);
    expect(filter.searchBox().val()).toBe('');
    expect(collection.at(0).id).toBe(1);
    expect(collection.at(1).id).toBe(2);
  });

  it("shows the clear button when the search box has text entered, hides it otherwise", function (done) {
    var filter = new Backgrid.Extension.LunrFilter({
      collection: collection,
      fields: {name: 1, bio: 10}
    });
    filter.render();
    filter.searchBox().val("bob").keydown().keyup();

    done();

    expect(collection.length).toBe(1);
    expect(filter.clearButton().css("display")).not.toBe("none");

    filter.searchBox().val(null).keydown().keyup();

    done();

    expect(collection.length).toBe(2);
    expect(filter.clearButton().css("display")).toBe("none");
  });

});
