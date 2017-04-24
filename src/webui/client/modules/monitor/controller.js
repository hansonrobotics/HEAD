define(['application', './views/layout', 'modules/motors/views/motors', './views/messages', './views/logs',
        './views/speed', './views/system', './entities/log_collection', './entities/ros_node_collection',
        'entities/motor_collection', './views/status', './views/processes'],
    function(App, LayoutView, MotorsView, MessagesView, LogsView, SpeedView, SystemView, LogCollection,
             RosNodeCollection, MotorCollection, StatusView) {
        return {
            init: function(name) {
                if (!this.layoutView) {
                    let self = this

                    this.layoutView = new LayoutView()
                    this.layoutView.on('destroy', function() {
                        delete self.layoutView
                    })

                    App.LayoutInstance.setTitle('Monitoring')
                    App.LayoutInstance.getRegion('content').show(this.layoutView)
                }

                this.layoutView.setActive(name)
            },
            motors: function() {
                this.init('motors')
                let motorsCollection = new MotorCollection(),
                    motorsView = new MotorsView({
                        collection: motorsCollection,
                        monitoring: true
                    })

                motorsCollection.fetchFromParam()

                this.layoutView.getRegion('content').show(motorsView)
            },
            messages: function() {
                this.init('messages')
                this.messagesView = new MessagesView()
                this.layoutView.getRegion('content').show(this.messagesView)
            },
            logs: function() {
                this.init('logs')
                this.logCollection = new LogCollection()
                this.logCollection.fetch()

                this.logsView = new LogsView({collection: this.logCollection})
                this.layoutView.getRegion('content').show(this.logsView)
            },
            processes: function() {
                this.init('processes')
                this.speedView = new SpeedView()
                this.layoutView.getRegion('content').show(this.speedView)
            },
            speed: function() {
                this.init('speed')
                this.speedView = new SpeedView()
                this.layoutView.getRegion('content').show(this.speedView)
            },
            system: function() {
                this.init('system')
                let rosNodeCollection = new RosNodeCollection()
                rosNodeCollection.fetch()
                this.systemView = new SystemView({
                    collection: rosNodeCollection
                })
                this.layoutView.getRegion('content').show(this.systemView)
            },
            status: function() {
                let statusView = new StatusView({})
                App.LayoutInstance.setTitle('Status')
                App.LayoutInstance.getRegion('content').show(statusView)
            }
        }
    })
