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
echo "PX4IO start failed" >> $LOG_FILE
tune_control play -t 20
fi
fi
