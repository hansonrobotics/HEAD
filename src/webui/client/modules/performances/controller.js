define(['application', 'lib/api', './views/layout', './views/attention_regions', './css/performances'],
    function(app, api, LayoutView, AttentionRegionsView) {
        return {
            performances: function(dir) {
                if (!this.layoutView || this.layoutView.isDestroyed()) {
                    this.layoutView = new LayoutView({
                        dir: dir,
                        nav: true,
                        disableSaving: true,
                        allowEdit: true
                    })

                    api.blenderMode.enable()
                    api.disableInteractionMode()

                    // show page
                    app.LayoutInstance.setTitle('Performances')
                    app.LayoutInstance.setFluid(true)
                    app.LayoutInstance.getRegion('content').show(this.layoutView)
                } else if (!app.skipPerformanceNav) {
                    this.layoutView.getRegion('performances').currentView.navigate(dir || '')
                } else {
                    app.skipPerformanceNav = false
                }
            },
            attention_regions: function() {
                let attentionRegionsView = new AttentionRegionsView()
                api.disableInteractionMode()

                // show page
                app.LayoutInstance.setTitle('Attention Regions')
                app.LayoutInstance.setFluid(false)
                app.LayoutInstance.getRegion('content').show(attentionRegionsView)
            }
        }
    })
