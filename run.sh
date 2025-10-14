#!/bin/bash
rm lib/all_defines.h
rm lib/all_enums.h
rm lib/inav_enums.py
rm lib/inav_defines.py
rm lib/msp_enum.py
rm lib/msp_messages.json
touch lib/inav_enums.py
touch lib/inav_defines.py
touch lib/msp_enum.py

echo "###########"
echo get_msp_enums.py
python get_msp_enums.py
read -n 1 -s -r -p "Press any key to continue"

echo "###########"
echo parsemd.py
python parsemd.py
read -n 1 -s -r -p "Press any key to continue"

echo "###########"
echo get_inav_defines.py
python get_inav_defines.py
read -n 1 -s -r -p "Press any key to continue"

echo "###########"
echo get_inav_enums.py
python get_inav_enums.py
read -n 1 -s -r -p "Press any key to continue"

#echo "###########"
#echo h_to_enum.py
#python h_to_enum.py
#echo "Go fix what's messed up in the headers"
#read -n 1 -s -r -p "Press any key to continue"

echo "###########"
echo bad_define_parse.py
python bad_define_parse.py
read -n 1 -s -r -p "Press any key to continue"


echo "###########"
echo mspref.py
python mspref.py
read -n 1 -s -r -p "Press any key to continue"


echo "###########"
echo get_all_inav_enums_h.py
python get_all_inav_enums_h

echo "###########"
echo gen_msp_md.py
python gen_msp_md.py

echo "###########"
echo gen_enum_doc.py
python gen_enum_doc.py
read -n 1 -s -r -p "Press any key to continue"