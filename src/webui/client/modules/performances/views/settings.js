define(['application', 'marionette', './templates/settings.tpl', 'lib/regions/fade_in', './attention_regions', 'lib/api', 'path'],
    function (App, Marionette, template, FadeInRegion, AttentionRegionsView, api, path) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                keywords: '.app-keywords',
                saveKeywordsButton: '.app-save-keywords'
            },
            regions: {
                selectAreas: {
                    el: '.app-select-areas-content',
                    regionClass: FadeInRegion
                }
            },
            events: {
                'click @ui.saveKeywordsButton': 'saveKeywords'
            },
            initialize: function (options) {
                this.mergeOptions(options, ['performancePath']);
                console.log(this);
            },
            onRender: function () {
                var self = this;
                $.ajax({
                    method: 'GET',
                    url: path.join('/keywords', this.performancePath),
                    success: function (data) {
                        self.ui.keywords.val(data['keywords'].join(', '));
                    }
                });
                this.getRegion('selectAreas').show(new AttentionRegionsView({performancePath: this.performancePath}));
            },
            saveKeywords: function () {
                var self = this;
                $.ajax({
                    method: 'POST',
                    url: path.join('/keywords', this.performancePath),
                    data: JSON.stringify({
                        path: this.performancePath,
                        keywords: this.ui.keywords.val().split(',')
                    }),
                    success: function () {
                        App.Utilities.showPopover(self.ui.saveKeywordsButton, 'Saved', 'right');
                    },
                    error: function () {
                        App.Utilities.showPopover(self.ui.saveKeywordsButton, 'Error', 'right');
                    }
                });
            }
        });
    });
