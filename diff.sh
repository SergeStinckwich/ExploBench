#!/bin/sh

explorestage=$(rospack find explore_stage)
explorebeego=$(pwd)/explore_beego
filediff=explore_stage_beego.diff
diff -ur $explorestage $explorebeego > $filediff
echo "gedit $filediff "

