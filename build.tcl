set project_root [file dirname [file normalize [info script]]]
open_project [file join $project_root pid_trial.gprj]
set_option -use_sspi_as_gpio 1
run all
