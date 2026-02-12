# LaneletWay.msg
# A way associated with one or more lanelets.
#
# Used for ref_lines (stop lines) in regulatory elements, where a single
# stop line may span multiple lanelets and consumers need to know which
# lanelets it applies to.

Way way
int64[] lanelet_ids
