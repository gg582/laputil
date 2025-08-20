#!/bin/bash
# Generates C array of AC adapter names for lap_is_on_ac patch

PS_DIR=/sys/class/power_supply
OUT_FILE=include/ac_names_gen.h

mkdir include
echo "// Auto-generated AC adapter names" > $OUT_FILE
echo "static const char *ac_names[] = {" >> $OUT_FILE

for d in $PS_DIR/*; do
    if [ -f "$d/type" ]; then
        type=$(cat "$d/type")
        if [ "$type" = "Mains" ]; then
            name=$(basename "$d")
            echo "    \"$name\"," >> $OUT_FILE
        fi
    fi
done

echo "    NULL" >> $OUT_FILE
echo "};" >> $OUT_FILE

echo "// Auto-generated Battery adapter names" >> $OUT_FILE
echo "static const char *battery_names[] = {" >> $OUT_FILE

for d in $PS_DIR/*; do
    if [ -f "$d/type" ]; then
        type=$(cat "$d/type")
        if [ "$type" = "Battery" ]; then
            name=$(basename "$d")
            echo "    \"$name\"," >> $OUT_FILE
        fi
    fi
done

echo "    NULL" >> $OUT_FILE
echo "};" >> $OUT_FILE

 
