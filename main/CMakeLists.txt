idf_component_register(SRCS "pid_ctrl.c" "bdc_motor.c" "bdc_motor_mcpwm_impl.c" "main.c"
	INCLUDE_DIRS "."
	EMBED_TXTFILES web/index.html
)
