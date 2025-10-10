#!/bin/bash
echo "###########"
echo get_msp_defs.py
python get_msp_defs.py

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

echo "Go fix what's messed up in the headers"