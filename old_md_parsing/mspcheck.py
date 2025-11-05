from lib.msp_enum import MSPCodes


## MSPv1 Baseflight/INAV Commands (64-99, plus others)
## MSPv1 MultiWii Original Commands (100-127, 130)
## MSPv1 MultiWii Original Input Commands (200-221)
## MSPv1 System & Debug Commands (239-254)
## MSPv1 Extended/INAV Commands (150-166)

## MSPv2 Common Commands (0x1000 Range)
## MSPv2 INAV Specific Commands (0x2000 Range)
## MSPv2 Betaflight Commands (0x3000 Range)
## MSPv2 Sensor Input Commands (0x1F00 Range)

def printer(i,j, print_undefined=False):
    for x in range(i,j):
        try:
            e = MSPCodes(x)
            print(f"{e.name:<32}\t{x}")
        except:
            if print_undefined: 
                print(f"{'UNDEFINED':<32}\t{x}")

print("MSPv1")
printer(0,254, True)
print("\nMSPv2")
printer(4096,20000, False)