if [ $OUTPUT_MODE = hil ]
then
set HIL_ARG $OUTPUT_MODE
fi
if [ $IO_PRESENT = yes ]
then
if px4io start $HIL_ARG
then
px4io recovery
else
echo "PX4IO start failed"
tune_control play -t 18 # PROG_PX4IO_ERR
fi
fi
