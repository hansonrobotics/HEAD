define(['marionette', './templates/crosshairs.tpl', 'lib/api', 'lib/crosshair-slider'],
    function (Marionette, template, api) {
        return Marionette.View.extend({
            template: template,
            ui: {
                headControl: '.app-head-control',
                eyeControl: '.app-eye-control'
            },
            speechInputKeyPress: function (e) {
                if (e.keyCode == 13) this.saySpeech();
            },
            onRender: function () {
                this.buildHeadCrosshair();
                this.buildEyeCrosshair();
            },
            buildHeadCrosshair: function () {
                this.ui.headControl.crosshairsl({
                    change: function (e, ui) {
                        api.setFaceTarget(1, ui.xval / 50, ui.yval / -50);
                    }
                });
            },
            buildEyeCrosshair: function () {
                this.ui.eyeControl.crosshairsl({
                    change: function (e, ui) {
                        api.setGazeTarget(1, ui.xval / 50, ui.yval / -50);
                    }
                });
            }
        });
    });
