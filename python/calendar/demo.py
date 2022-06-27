import calendar
import pdb
pdb.set_trace()

print(calendar.prcal(2022, m = 4))

cal = calendar.Calendar(0)

num = 0
MOD = 3
for month in range(1, 13):
    print(f"Month: {month}")
    print("    Mo      Tu    We   Th   Fr   Sa   Su")
    output_str = ""
    for item in cal.itermonthdays4(2022, month = month):
        if item[1] == month:
            if item[3] < 5:
                # weekday
                mod = num % MOD
                mod1 = (mod + 1) % MOD
                mod2 = (mod +2) % MOD
                output_str += f"{{{mod + 1},{mod1 + 1},{mod2 + 1},{mod + 1}}}"
                num += 1
            else:
                output_str += "        "
                if item[3] == 6:
                    print(output_str)
                    output_str = ""
                pass
        else:
            output_str += "        "
            pass