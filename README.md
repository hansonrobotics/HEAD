##basic_head_api

Provides `/make_face_expr` service to call with a **MakeFaceExpr.srv**, the request of which includes a face expression string and intensity. The service sends appropriate **servo_pololu** messages on `/cmd_pololu` and responds with corresponding motor positions.

See **einstein.yaml** for available expression strings to include inside the **MakeFaceExpr.srv** request. Or get them at runtime with `/valid_face_exprs` and **ValidFaceExprs.srv** service.

###Notes

Currently only supports Mini-Einstein.

TODO: Save face expressions in a robot-agnostic way, possibly in Blender.
