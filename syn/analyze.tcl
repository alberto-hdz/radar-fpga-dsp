# Re-open the routed design and dump every failing setup path (1 per endpoint).
# Run from the syn/ folder after build.tcl.
open_checkpoint out/post_route.dcp
report_timing -setup -max_paths 5000 -nworst 1 -slack_lesser_than 0 -file out/failing.rpt
