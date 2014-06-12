##basic_head_api

Listens to `/cmd_face_expr` topic for **FaceExprReq** messages, which include a face expression string and intensity, and sends appropriate **servo_pololu** messages on `/cmd_pololu`.

See **einstein.yaml** for available expression strings to include inside **FaceExprReq**.

###Notes

Currently only supports Mini-Einstein.

TODO: Save face expressions in a robot-agnostic way, possibly in Blender.
