#!/bin/bash
rm lib/all_defines.h
rm lib/inav_enums.py
rm lib/inav_defines.py
rm lib/msp_enum.py
rm lib/msp_messages.json

echo "###########"
echo get_msp_enums.py
python get_msp_enums.py

echo "###########"
echo parsemd.py
python parsemd.py

echo "###########"
echo get_inav_defines.py
python get_inav_defines.py

echo "###########"
echo get_inav_enums.py
python get_inav_enums.py

#echo "###########"
#echo h_to_enum.py
#python h_to_enum.py
#echo "Go fix what's messed up in the headers"

echo "###########"
echo bad_define_parse.py
python bad_define_parse.py


echo "###########"
echo mspref.py
python mspref.py
