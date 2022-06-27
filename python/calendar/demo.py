import calendar
from calendar import monthrange
import pdb
import logging
import sys
# pdb.set_trace()

# print(calendar.prcal(2022, m = 4))

logging.basicConfig(format='%(asctime)s.%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.DEBUG)

cal = calendar.Calendar(0)

num = 0
if len(sys.argv) > 1:
    num = int(sys.argv[1])
    logging.info(f"setting to work out the {num + 1}'s order")

if num > 2:
    logging.warnging(f"first argument can not be larger than 2, which is {num}, should 0, 1 or 2. Setting it to 0")
    num = 0


MOD = 3
year = 2022
for month in range(1, 13):
    print(f"Month: {month}")
    print("Mo  Tu  We  Th  Fr  Sa  Su")
    last_day = monthrange(year, month)[1]
    logging.debug(f"last day is {last_day} when in {month} of {year}")
    output_str = ""
    for item in cal.itermonthdays4(year, month = month):
        if item[1] == month:
            if item[3] < 5:
                # weekday
                mod = num % MOD
                mod1 = (mod + 1) % MOD
                mod2 = (mod + 2) % MOD
                num += 1
                # logging.debug(f"num is {num} after {item}")
                output_str += f" {mod + 1}, "
            else:
                output_str += "  "
                pass
            if item[3] == 6 or item[2] == last_day:
                print(output_str)
                output_str = ""
            pass


        else:
            output_str += "    "
            pass
